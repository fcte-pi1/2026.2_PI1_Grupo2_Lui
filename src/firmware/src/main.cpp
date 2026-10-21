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
#include "navigation/PositionTracker.h"
#include "navigation/RobotStateEstimator.h"
#include "navigation/WallPerception.h"
#include "hal/IWallSensor.h"
#include "hal/IOdometryReader.h"
#include "hal/IOrientationSensor.h"

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

// O sampler de hardware é a única camada que dispara os ultrassons. O estimador
// e o alinhamento PID consomem o mesmo snapshot, sem leituras duplicadas.
struct LateralSnapshot {
    UltrasonicReading front{false, NAN, 0};
    UltrasonicReading left{false, NAN, 0};
    UltrasonicReading right{false, NAN, 0};
    uint32_t sampledAtMs = 0;
    bool ready = false;
};

portMUX_TYPE lateralMux = portMUX_INITIALIZER_UNLOCKED;
LateralSnapshot lateralSnapshot;

LateralSnapshot getLateralSnapshot() {
    LateralSnapshot copy;
    portENTER_CRITICAL(&lateralMux);
    copy = lateralSnapshot;
    portEXIT_CRITICAL(&lateralMux);
    return copy;
}

class HardwareWallSensor final : public perception::IWallSensor {
public:
    perception::RawEcho read(perception::Direction direction) override {
        // WallPerception solicita sempre Frente, Esquerda e Direita nessa ordem.
        // Reutilizar um único snapshot evita combinar sensores de dois ciclos se
        // a task de aquisição publicar uma amostra durante esta leitura.
        if (direction == perception::Direction::Front || !hasCachedSnapshot_) {
            cachedSnapshot_ = getLateralSnapshot();
            hasCachedSnapshot_ = true;
        }
        const LateralSnapshot& snapshot = cachedSnapshot_;
        const UltrasonicReading* reading = &snapshot.front;
        if (direction == perception::Direction::Left) reading = &snapshot.left;
        else if (direction == perception::Direction::Right) reading = &snapshot.right;

        // O pulso zero é ausência de eco (via livre) segundo o contrato HU-01.
        // A validação física e a suavização ficam centralizadas em WallPerception.
        perception::RawEcho echo{};
        echo.timestampMs = snapshot.sampledAtMs;
        if (reading->pulseUs == 0) {
            echo.hasEcho = false;
            echo.rawDistanceMm = 0;
        } else {
            const uint32_t distanceMm =
                (static_cast<uint32_t>(reading->pulseUs) * 10U + 29U) / 58U;
            echo.rawDistanceMm = static_cast<uint16_t>(
                distanceMm > 65535U ? 65535U : distanceMm);
            echo.hasEcho = true;
        }
        if (direction == perception::Direction::Right) hasCachedSnapshot_ = false;
        return echo;
    }

private:
    LateralSnapshot cachedSnapshot_{};
    bool hasCachedSnapshot_ = false;
};

class MotionDistanceReader final : public tracking::IOdometryReader {
public:
    explicit MotionDistanceReader(MotionController& motion) : motion_(motion) {}

    tracking::OdometryReading read() override {
        const float delta = currentMoveDistance() - accumulatorBaselineMm_;
        float bounded = delta;
        if (bounded > 32767.0f) bounded = 32767.0f;
        if (bounded < -32768.0f) bounded = -32768.0f;
        return {static_cast<int16_t>(lroundf(bounded)), millis()};
    }

    void resetAccumulator() override { accumulatorBaselineMm_ = currentMoveDistance(); }
    void beginMotionSegment() { accumulatorBaselineMm_ = currentMoveDistance(); }

private:
    float currentMoveDistance() const {
        return motion_.getMotionType() == MotionType::ForwardCell
            ? motion_.getDistanceTravelledMm() : 0.0f;
    }

    MotionController& motion_;
    float accumulatorBaselineMm_ = 0.0f;
};

class ClockwiseYawSensor final : public tracking::IOrientationSensor {
public:
    explicit ClockwiseYawSensor(IMUMPU6500& imu) : imu_(imu) {}

    tracking::OrientationReading read() override {
        // IMU: yaw positivo anti-horário. Grade: N->E->S->W horário.
        float yaw = -imu_.getYawDegrees();
        while (yaw > 180.0f) yaw -= 360.0f;
        while (yaw < -180.0f) yaw += 360.0f;
        return {static_cast<int16_t>(lroundf(yaw)), millis()};
    }

private:
    IMUMPU6500& imu_;
};

HardwareWallSensor hardwareWallSensor;
MotionDistanceReader gridOdometry(motion);
ClockwiseYawSensor gridOrientation(imu);
tracking::PositionTracker positionTracker(gridOdometry, gridOrientation);
perception::WallPerception wallPerception(hardwareWallSensor);
state_estimation::RobotStateEstimator robotStateEstimator(positionTracker, wallPerception);

MazeConfig mazeConfig{};
mm::MazeMap mazeMap;
mm::NvsMapStorage mapStorage;
nav::MazeMissionController mission(mazeMap, mapStorage, robotStateEstimator);

bool ready = false;
bool motionResultPending = false;
uint32_t attemptStartedMs = 0;
uint32_t lastControlUpdateUs = 0;
uint32_t observationStartedAtMs = 0;
uint32_t lastProcessedSensorTimestampMs = 0;
uint8_t freshObservationSamples = 0;

bool isSensorSnapshotReady() {
    return getLateralSnapshot().ready;
}

void publishSensorSnapshot(const UltrasonicReading& front,
                           const UltrasonicReading& left,
                           const UltrasonicReading& right,
                           uint32_t sampleStartedAtMs) {
    portENTER_CRITICAL(&lateralMux);
    lateralSnapshot.front = front;
    lateralSnapshot.left = left;
    lateralSnapshot.right = right;
    lateralSnapshot.sampledAtMs = sampleStartedAtMs;
    lateralSnapshot.ready = true;
    portEXIT_CRITICAL(&lateralMux);
}

// Executa no núcleo de sensores; pulseIn() nunca bloqueia o ciclo PID principal.
void lateralSensorTask(void*) {
    for (;;) {
        const uint32_t sampleStartedAtMs = millis();
        const UltrasonicReading front = sensorFront.measure();
        vTaskDelay(pdMS_TO_TICKS(1)); // separação para reduzir diafonia acústica
        const UltrasonicReading left = sensorLeft.measure();
        vTaskDelay(pdMS_TO_TICKS(1));
        const UltrasonicReading right = sensorRight.measure();
        publishSensorSnapshot(front, left, right, sampleStartedAtMs);
        vTaskDelay(pdMS_TO_TICKS(LATERAL_SENSOR_PERIOD_MS));
    }
}

float initialHeadingRadians(Direction heading) {
    constexpr float kHalfPiRadians = 1.57079632679489661923f;
    constexpr float kPiRadians = 3.14159265358979323846f;
    switch (heading) {
        case Direction::East: return 0.0f;
        case Direction::North: return kHalfPiRadians;
        case Direction::West: return kPiRadians;
        case Direction::South: return -kHalfPiRadians;
    }
    return 0.0f;
}

void safeStop(const char *reason) {
    ready = false;
    motion.stop();
    motors.brake();
    motors.setEnabled(false);
    Serial.printf("Tentativa interrompida: %s\n", reason);
}

void handleMissionUpdate(const nav::MissionUpdate &update) {
    bool started = false;
    switch (update.action) {
        case nav::MissionAction::None:
            return;
        case nav::MissionAction::MoveForwardOneCell:
            started = motion.startMoveForwardOneCell();
            break;
        case nav::MissionAction::MoveBackwardOneCell:
            started = motion.startMoveBackwardOneCell();
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
        const nav::MissionUpdate failed = mission.onMotionFinished(false);
        (void)failed;
        safeStop("não foi possível iniciar a ação de movimento");
        return;
    }
    gridOdometry.beginMotionSegment();
    motionResultPending = true;
    lastControlUpdateUs = micros();
    // O sampler lê os três sensores continuamente; o PID e a HU-01 consomem o mesmo snapshot.
}

void updateWallAlignmentFeedback() {
    const LateralSnapshot snapshot = getLateralSnapshot();
    const uint32_t age = static_cast<uint32_t>(millis() - snapshot.sampledAtMs);
    const bool valid = snapshot.ready && snapshot.left.valid && snapshot.right.valid &&
                       age <= MAX_LATERAL_SAMPLE_AGE_MS;
    if (valid) {
        motion.setWallAlignmentFeedback(snapshot.left.distanceCm * 10.0f,
                                        snapshot.right.distanceCm * 10.0f, true);
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

    const bool i2cReady = Wire.begin(
        BoardPins::I2C_SDA,
        BoardPins::I2C_SCL
    );

    if (!i2cReady) {
        Serial.println(
            "[ERRO] Falha ao inicializar o barramento I2C."
        );

        return;
    }
    
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
    
    // HU-02: verifica se a IMU responde antes de tentar ler seus registradores.
    Wire.beginTransmission(BoardPins::MPU6500_ADDRESS);

    const uint8_t imuProbeStatus = Wire.endTransmission(true);

    bool imuOK = false;

    if (imuProbeStatus == 0) {
        Serial.println("[OK] MPU6500 respondeu no barramento I2C.");

        imuOK = imu.begin(Wire, BoardPins::MPU6500_ADDRESS);

        if (!imuOK) {
            Serial.println(
                "[ERRO] A MPU6500 respondeu, mas sua inicializacao falhou."
            );
        }
    } else {
        Serial.printf(
            "[INFO] MPU6500 nao detectada em 0x%02X (status I2C=%u).\n",
            BoardPins::MPU6500_ADDRESS,
            static_cast<unsigned>(imuProbeStatus)
        );
    }
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

    if (!mission.begin(mazeConfig, gridOrientation.read().yawDegrees)) {
        Serial.println("Falha ao inicializar o mapa/navegação da missão.");
        motors.setEnabled(false);
        return;
    }

    if (xTaskCreatePinnedToCore(lateralSensorTask, "lateral-sensors", 4096,
                                nullptr, 1, nullptr, 0) != pdPASS) {
        Serial.println("Falha ao iniciar a task dos sensores.");
        motors.setEnabled(false);
        return;
    }

    observationStartedAtMs = millis();
    attemptStartedMs = observationStartedAtMs;
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

    if (isSensorSnapshotReady()) {
        const LateralSnapshot sensorSnapshot = getLateralSnapshot();
        if (sensorSnapshot.sampledAtMs != lastProcessedSensorTimestampMs) {
            lastProcessedSensorTimestampMs = sensorSnapshot.sampledAtMs;
            robotStateEstimator.updateWalls();
            if (mission.state() == nav::MissionState::AwaitingObservation &&
                static_cast<int32_t>(sensorSnapshot.sampledAtMs - observationStartedAtMs) >= 0 &&
                freshObservationSamples < 3) {
                ++freshObservationSamples;
            }
        }
    }

    if (motionResultPending) {
        MotionStatus status = motion.getStatus();
        if (status == MotionStatus::Running) {
            const uint32_t nowUs = micros();
            const uint32_t elapsedUs = static_cast<uint32_t>(nowUs - lastControlUpdateUs);
            if (elapsedUs >= CONTROL_PERIOD_US) {
                motion.update(static_cast<float>(elapsedUs) / 1000000.0f);
                robotStateEstimator.updatePose();
                lastControlUpdateUs = nowUs;
            }
            if (motion.getStatus() == MotionStatus::Running) {
                updateWallAlignmentFeedback();
                delayMicroseconds(500); // evita polling contínuo sem atrasar o ciclo PID
                return;
            }
            status = motion.getStatus();
        }

        motionResultPending = false;
        const nav::MissionUpdate missionUpdate =
            mission.onMotionFinished(status == MotionStatus::Succeeded);
        handleMissionUpdate(missionUpdate);
        if (missionUpdate.state == nav::MissionState::AwaitingObservation) {
            observationStartedAtMs = millis();
            freshObservationSamples = 0;
        }
        return;
    }

    if (mission.state() == nav::MissionState::AwaitingObservation) {
        const auto& state = robotStateEstimator.currentState();
        const uint32_t sampleAge = static_cast<uint32_t>(millis() - state.sensorTimestampMs);
        if (!state.initialized || freshObservationSamples < 3 ||
            sampleAge > MAX_LATERAL_SAMPLE_AGE_MS) {
            delayMicroseconds(500);
            return;
        }
        handleMissionUpdate(mission.observeCurrentCell());
    }
}
