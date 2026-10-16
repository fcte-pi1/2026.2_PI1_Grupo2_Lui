#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config/board_pins.hpp"
#include "config/maze_config.hpp"
#include "hal/encoder_n20.hpp"
#include "hal/imu_mpu6500.hpp"
#include "hal/motor_drv8833.hpp"
#include "hal/ultrasonic_us100.hpp"
#include "hal/NvsMapStorage.h"
#include "control/motion_controller.hpp"
#include "control/odometry.hpp"
#include "control/pid_controller.hpp"
#include "navigation/MazeMap.h"
#include "navigation/MazeMissionController.h"

namespace {

// Estes valores são a configuração única desta tentativa. O dashboard poderá
// substituí-los futuramente; a origem/objetivo não são inferidos dos sensores.
constexpr MazeFormat SELECTED_MAZE_FORMAT = MazeFormat::Maze4x4;
constexpr MazeCorner SELECTED_START_CORNER = MazeCorner::TopLeft;
constexpr Direction INITIAL_HEADING = Direction::East;
constexpr uint32_t CONTROL_PERIOD_US = 5000UL;
constexpr uint32_t LATERAL_SENSOR_PERIOD_MS = 20UL;
constexpr uint32_t MAX_LATERAL_SAMPLE_AGE_MS = 150UL;

MotorDRV8833 motors;
EncoderN20 encoderLeft;
EncoderN20 encoderRight;
IMUMPU6500 imu;
UltrasonicUS100 sensorFront;
UltrasonicUS100 sensorLeft;
UltrasonicUS100 sensorRight;

Odometry odometry;
PIDController leftPid(0.8f, 0.10f, 0.0f, -120.0f, 120.0f);
PIDController rightPid(0.8f, 0.10f, 0.0f, -120.0f, 120.0f);
MotionController motion(motors, encoderLeft, encoderRight, imu, odometry,
                        leftPid, rightPid);

MazeConfig mazeConfig{};
mm::MazeMap mazeMap;
mm::NvsMapStorage mapStorage;
nav::MazeMissionController mission(mazeMap, mapStorage);

bool ready = false;
bool motionResultPending = false;
uint32_t attemptStartedMs = 0;
uint32_t lastControlUpdateUs = 0;

struct LateralSnapshot {
    float leftMm = 0.0f;
    float rightMm = 0.0f;
    uint32_t sampledAtMs = 0;
    bool valid = false;
};

portMUX_TYPE lateralMux = portMUX_INITIALIZER_UNLOCKED;
bool lateralTaskEnabled = false;
bool lateralTaskBusy = false;
LateralSnapshot lateralSnapshot;

bool lateralShouldStartRead() {
    bool start = false;
    portENTER_CRITICAL(&lateralMux);
    if (lateralTaskEnabled && !lateralTaskBusy) {
        lateralTaskBusy = true;
        start = true;
    }
    portEXIT_CRITICAL(&lateralMux);
    return start;
}

void setLateralTaskEnabled(bool enabled) {
    portENTER_CRITICAL(&lateralMux);
    lateralTaskEnabled = enabled;
    if (!enabled) lateralSnapshot.valid = false;
    portEXIT_CRITICAL(&lateralMux);
}

bool isLateralTaskBusy() {
    bool busy;
    portENTER_CRITICAL(&lateralMux);
    busy = lateralTaskBusy;
    portEXIT_CRITICAL(&lateralMux);
    return busy;
}

void publishLateralSnapshot(const UltrasonicReading &left,
                            const UltrasonicReading &right) {
    portENTER_CRITICAL(&lateralMux);
    lateralSnapshot.valid = left.valid && right.valid;
    if (lateralSnapshot.valid) {
        lateralSnapshot.leftMm = left.distanceCm * 10.0f;
        lateralSnapshot.rightMm = right.distanceCm * 10.0f;
    }
    lateralSnapshot.sampledAtMs = millis();
    lateralTaskBusy = false;
    portEXIT_CRITICAL(&lateralMux);
}

LateralSnapshot getLateralSnapshot() {
    LateralSnapshot copy;
    portENTER_CRITICAL(&lateralMux);
    copy = lateralSnapshot;
    portEXIT_CRITICAL(&lateralMux);
    return copy;
}

// Leituras laterais ocorrem numa task separada para não bloquear o ciclo PID
// de 5 ms durante pulseIn() do US-100. A leitura frontal é feita em cada centro.
void lateralSensorTask(void *) {
    for (;;) {
        if (!lateralShouldStartRead()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        const UltrasonicReading left = sensorLeft.measure();
        const UltrasonicReading right = sensorRight.measure();
        publishLateralSnapshot(left, right);
        vTaskDelay(pdMS_TO_TICKS(LATERAL_SENSOR_PERIOD_MS));
    }
}

float initialHeadingRadians(Direction heading) {
    constexpr float HALF_PI = 1.57079632679489661923f;
    constexpr float PI = 3.14159265358979323846f;
    switch (heading) {
        case Direction::East: return 0.0f;
        case Direction::North: return HALF_PI;
        case Direction::West: return PI;
        case Direction::South: return -HALF_PI;
    }
    return 0.0f;
}

bool classifyWall(const UltrasonicReading &reading, bool &hasWall) {
    if (!reading.valid) return false;
    const float thresholdMm = mazeConfig.cellSizeMm *
        MazeSettings::WALL_THRESHOLD_CELL_RATIO;
    hasWall = reading.distanceCm * 10.0f <= thresholdMm;
    return true;
}

bool readWallsAtCell(nav::WallReadings &walls) {
    // A task de sensores laterais precisa terminar antes que a mesma HAL seja
    // usada pela rotina de observação do centro da célula.
    if (isLateralTaskBusy()) return false;
    const UltrasonicReading front = sensorFront.measure();
    const UltrasonicReading left = sensorLeft.measure();
    const UltrasonicReading right = sensorRight.measure();
    return classifyWall(front, walls.front) &&
           classifyWall(left, walls.left) &&
           classifyWall(right, walls.right);
}

void safeStop(const char *reason) {
    setLateralTaskEnabled(false);
    ready = false;
    motion.stop();
    motors.brake();
    motors.setEnabled(false);
    Serial.printf("Tentativa interrompida: %s\n", reason);
}

void handleMissionUpdate(const nav::MissionUpdate &update) {
    bool started = false;
    bool needsLateralFeedback = false;
    switch (update.action) {
        case nav::MissionAction::None:
            return;
        case nav::MissionAction::MoveForwardOneCell:
            started = motion.startMoveForwardOneCell();
            needsLateralFeedback = true;
            break;
        case nav::MissionAction::TurnLeft90:
            started = motion.startTurnLeft90();
            break;
        case nav::MissionAction::TurnRight90:
            started = motion.startTurnRight90();
            break;
        case nav::MissionAction::TurnAround180:
            started = motion.startTurnAround180();
            break;
        case nav::MissionAction::Brake:
            setLateralTaskEnabled(false);
            ready = false;
            motors.brake();
            motors.setEnabled(false);
            if (update.state == nav::MissionState::Completed) {
                Serial.printf("Objetivo alcançado em (%u, %u). Mapa final salvo.\n",
                              mission.x(), mission.y());
            } else {
                Serial.printf("Falha de navegação (status=%u); motores parados.\n",
                              static_cast<unsigned>(update.navigationStatus));
            }
            return;
    }

    if (!started) {
        setLateralTaskEnabled(false);
        const nav::MissionUpdate failed = mission.onMotionFinished(false);
        (void)failed;
        safeStop("não foi possível iniciar a ação de movimento");
        return;
    }
    motionResultPending = true;
    lastControlUpdateUs = micros();
    setLateralTaskEnabled(needsLateralFeedback);
}

void updateWallAlignmentFeedback() {
    const LateralSnapshot snapshot = getLateralSnapshot();
    const uint32_t age = static_cast<uint32_t>(millis() - snapshot.sampledAtMs);
    if (snapshot.valid && age <= MAX_LATERAL_SAMPLE_AGE_MS) {
        motion.setWallAlignmentFeedback(snapshot.leftMm, snapshot.rightMm, true);
    } else {
        motion.setWallAlignmentFeedback(0.0f, 0.0f, false);
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(100);

    mazeConfig = makeMazeConfig(SELECTED_MAZE_FORMAT,
                                SELECTED_START_CORNER,
                                INITIAL_HEADING);
    if (!isValidMazeConfig(mazeConfig)) {
        Serial.println("Configuração de labirinto inválida.");
        return;
    }

    Wire.begin(BoardPins::I2C_SDA, BoardPins::I2C_SCL);
    Wire.setClock(BoardPins::I2C_CLOCK_HZ);

    const bool motorsOK = motors.begin(
        BoardPins::MOTOR_LEFT.in1, BoardPins::MOTOR_LEFT.in2,
        BoardPins::MOTOR_RIGHT.in1, BoardPins::MOTOR_RIGHT.in2,
        BoardPins::MOTOR_NSLEEP, BoardPins::MOTOR_PWM_HZ,
        BoardPins::MOTOR_PWM_BITS);
    const bool leftEncoderOK = encoderLeft.begin(
        BoardPins::ENCODER_LEFT_A, BoardPins::ENCODER_LEFT_B);
    const bool rightEncoderOK = encoderRight.begin(
        BoardPins::ENCODER_RIGHT_A, BoardPins::ENCODER_RIGHT_B);
    const bool imuOK = imu.begin(Wire, BoardPins::MPU6500_ADDRESS);
    const bool frontSensorOK = sensorFront.begin(
        BoardPins::ULTRASONIC_FRONT.trigger, BoardPins::ULTRASONIC_FRONT.echo);
    const bool leftSensorOK = sensorLeft.begin(
        BoardPins::ULTRASONIC_LEFT.trigger, BoardPins::ULTRASONIC_LEFT.echo);
    const bool rightSensorOK = sensorRight.begin(
        BoardPins::ULTRASONIC_RIGHT.trigger, BoardPins::ULTRASONIC_RIGHT.echo);

    if (!motorsOK || !leftEncoderOK || !rightEncoderOK || !imuOK ||
        !frontSensorOK || !leftSensorOK || !rightSensorOK) {
        Serial.println("Falha na inicialização dos drivers/sensores.");
        motors.setEnabled(false);
        return;
    }

    esp_err_t nvsStatus = nvs_flash_init();
    if (nvsStatus == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvsStatus == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK || nvs_flash_init() != ESP_OK) {
            Serial.println("Falha ao inicializar NVS para persistência do mapa.");
            motors.setEnabled(false);
            return;
        }
    } else if (nvsStatus != ESP_OK) {
        Serial.println("Falha ao inicializar NVS para persistência do mapa.");
        motors.setEnabled(false);
        return;
    }

    Serial.println("Calibrando IMU. Mantenha o robô parado.");
    if (!imu.calibrateGyroZ()) {
        Serial.println("Falha na calibração da IMU.");
        motors.setEnabled(false);
        return;
    }

    MotionControllerConfig motionConfig;
    // Os encoders e parâmetros mecânicos ainda exigem calibração no robô real.
    motionConfig.leftCountsPerMm = 6.74f;
    motionConfig.rightCountsPerMm = 6.74f;
    motionConfig.wheelTrackMm = 90.0f;
    motionConfig.cellSizeMm = static_cast<float>(mazeConfig.cellSizeMm);
    motionConfig.imuYawSign = 1.0f;
    motionConfig.leftFeedForwardPwmPerMmS = 0.8f;
    motionConfig.rightFeedForwardPwmPerMmS = 0.8f;
    motionConfig.leftStaticFrictionPwm = 20.0f;
    motionConfig.rightStaticFrictionPwm = 20.0f;

    const Pose2D initialPose = {0.0f, 0.0f,
                                initialHeadingRadians(mazeConfig.initialHeading)};
    if (!motion.begin(motionConfig, initialPose)) {
        Serial.println("Falha ao inicializar o MotionController.");
        motors.setEnabled(false);
        return;
    }

    if (!mission.begin(mazeConfig)) {
        Serial.println("Falha ao inicializar o mapa/navegação da missão.");
        motors.setEnabled(false);
        return;
    }

    if (xTaskCreatePinnedToCore(lateralSensorTask, "lateral-sensors", 4096,
                                nullptr, 1, nullptr, 0) != pdPASS) {
        Serial.println("Falha ao iniciar a task dos sensores laterais.");
        motors.setEnabled(false);
        return;
    }

    attemptStartedMs = millis();
    motors.setEnabled(true);
    ready = true;
    Serial.printf("Labirinto %ux%u; início=(%u,%u); objetivo=(%u,%u).\n",
                  mazeConfig.rows, mazeConfig.cols,
                  mazeConfig.start.row, mazeConfig.start.col,
                  mazeConfig.goal.row, mazeConfig.goal.col);
    Serial.println("Navegação autônoma iniciada; sem intervenção manual durante a tentativa.");
}

void loop() {
    if (!ready) {
        delay(2);
        return;
    }

    if (static_cast<uint32_t>(millis() - attemptStartedMs) >=
        mazeConfig.attemptTimeoutMs) {
        mission.fail();
        safeStop("tempo máximo da tentativa excedido");
        return;
    }

    if (motionResultPending) {
        MotionStatus status = motion.getStatus();
        if (status == MotionStatus::Running) {
            const uint32_t nowUs = micros();
            const uint32_t elapsedUs = static_cast<uint32_t>(nowUs - lastControlUpdateUs);
            if (elapsedUs >= CONTROL_PERIOD_US) {
                motion.update(static_cast<float>(elapsedUs) / 1000000.0f);
                lastControlUpdateUs = nowUs;
            }
            if (motion.getStatus() == MotionStatus::Running) {
                updateWallAlignmentFeedback();
                delayMicroseconds(500); // evita polling contínuo sem atrasar o ciclo PID
                return;
            }
            status = motion.getStatus();
        }

        // Pausa a task de ultrassom antes de executar a próxima leitura de célula.
        setLateralTaskEnabled(false);
        if (isLateralTaskBusy()) {
            delayMicroseconds(500);
            return;
        }
        motionResultPending = false;
        handleMissionUpdate(mission.onMotionFinished(status == MotionStatus::Succeeded));
        return;
    }

    if (mission.state() == nav::MissionState::AwaitingObservation) {
        setLateralTaskEnabled(false);
        if (isLateralTaskBusy()) {
            delayMicroseconds(500);
            return;
        }
        nav::WallReadings walls{};
        if (!readWallsAtCell(walls)) {
            handleMissionUpdate(mission.fail());
            return;
        }
        handleMissionUpdate(mission.observeCell(walls));
    }
}
