#include <Arduino.h>
#include <ArduinoJson.h>
#include "communication/RemoteCommand.h"
#include "navigation/SessionRecorder.h"
#include <atomic>
#include <Wire.h>
#include <math.h>
#include <nvs_flash.h>
#include <esp_random.h>
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config/board_pins.hpp"
#include "config/mechanical_config.hpp"
#include "config/sensor_calibration.hpp"
#include "config/sonar_sampling.hpp"
#include "config/run_config.hpp"
#include "control/start_delay.hpp"
#include "control/front_echo_recovery.hpp"
#include "control/front_approach_policy.hpp"
#include "control/cell_observation_gate.hpp"
#include "control/front_proximity_guard.hpp"
#include "hal/SnapshotWallSensor.h"
#include "control/corridor_feedback_filter.hpp"
#include "config/maze_config.hpp"
#include "config/network_config.hpp"
#include "communication/TelemetryClient.h"
#include "hal/encoder_n20.hpp"
#include "hal/imu_mpu6500.hpp"
#include "hal/ina226.hpp"
#include "hal/motor_drv8833.hpp"
#include "hal/ultrasonic_us100.hpp"
#include "hal/NvsMapStorage.h"
#include "control/motion_controller.hpp"
#include "control/odometry.hpp"
#include "control/pid_controller.hpp"
#include "navigation/MazeMap.h"
#include "navigation/GuidedRoute.h"
#include "navigation/MazeMissionController.h"
#include "navigation/PositionTracker.h"
#include "navigation/RobotStateEstimator.h"
#include "navigation/WallPerception.h"
#include "hal/IWallSensor.h"
#include "hal/IOdometryReader.h"
#include "hal/IOrientationSensor.h"

namespace {

constexpr uint32_t CONTROL_PERIOD_US = 5000UL;
constexpr uint32_t MAX_LATERAL_SAMPLE_AGE_MS = SonarSampling::MAX_SAMPLE_AGE_MS;
CorridorFeedbackFilter3 corridorFeedbackFilter;
std::atomic<bool> samplingEnabled{true};
std::atomic<bool> samplingPaused{false};

MotorDRV8833 motors;
EncoderN20 encoderLeft;
EncoderN20 encoderRight;
IMUMPU6500 imu;
INA226 powerMonitor;
UltrasonicUS100 sensorFront;
UltrasonicUS100 sensorLeft;
UltrasonicUS100 sensorRight;
mm::TelemetryClient telemetryClient;

mm::SessionBuffer routeBuffer;
mm::SessionRecorder routeRecorder(routeBuffer);
bool telemetryClientReady = false;
bool remoteMissionActive = false;
bool remoteWasConnected = false;
const char* remoteState = "ready";
char lastStopReason[128]{};
uint32_t peakSensorAgeMs=0;
uint32_t remoteBootId = 0, lastRobotStatusMs = 0;
char previousStartId[33]{};
char remoteInput[256]{};
size_t remoteInputLength = 0;
bool droppingRemoteInput = false;

bool terminalTelemetryPublished = false;
uint32_t telemetrySessionId = 0;
uint32_t telemetrySessionStartedMs = 0;
uint32_t lastTelemetryPublishMs = 0;
uint32_t lastPowerSampleMs = 0;
bool powerMonitorReady = false;
bool powerReadingValid = false;
INA226Reading latestPowerReading{};

Odometry odometry;
PIDController leftPid(1.5f, 5.0f, 0.0f, -80.0f, 100.0f);
PIDController rightPid(1.5f, 5.0f, 0.0f, -80.0f, 100.0f);
MotionController motion(motors, encoderLeft, encoderRight, imu, odometry,
                        leftPid, rightPid);

// O sampler de hardware é a única camada que dispara os ultrassons. O estimador
// e o alinhamento PID consomem o mesmo snapshot, sem leituras duplicadas.
using LateralSnapshot = SonarSnapshot;

portMUX_TYPE lateralMux = portMUX_INITIALIZER_UNLOCKED;
LateralSnapshot lateralSnapshot;

LateralSnapshot getLateralSnapshot() {
    LateralSnapshot copy;
    portENTER_CRITICAL(&lateralMux);
    copy = lateralSnapshot;
    portEXIT_CRITICAL(&lateralMux);
    return copy;
}

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

SnapshotWallSensor hardwareWallSensor(getLateralSnapshot, []() -> uint32_t { return millis(); });
MotionDistanceReader gridOdometry(motion);
ClockwiseYawSensor gridOrientation(imu);
tracking::PositionTracker positionTracker(gridOdometry, gridOrientation);
perception::WallPerception wallPerception(hardwareWallSensor);
state_estimation::RobotStateEstimator robotStateEstimator(positionTracker, wallPerception);

MazeConfig mazeConfig{};
mm::MazeMap mazeMap;
mm::NvsMapStorage mapStorage;
nav::MazeMissionController mission(mazeMap, mapStorage, robotStateEstimator);

StartDelay startDelay;
FrontEchoRecovery frontEchoRecovery;
FrontProximityGuard frontProximity;
uint16_t frontEchoRecoveries=0;
bool ready = false;
bool hardwareReady = false;
bool attemptUsed = false;
bool guidedRouteActive = false;
bool guidedFrontApproach = false;
bool automaticFrontReferenceUsed=false;
bool cornerContinuationUsed = false;
nav::GuidedRoute guidedRoute;
uint32_t guidedPauseStartedMs = 0;
MotionControllerConfig motionConfig;
bool motionResultPending = false;
uint32_t attemptStartedMs = 0;
uint32_t lastControlUpdateUs = 0;
uint32_t lastProcessedSensorTimestampMs = 0;
CellObservationGate cellObservation;

void publishSensorSnapshot(const UltrasonicReading& front,
                           const UltrasonicReading& left,
                           const UltrasonicReading& right,
                           uint32_t sampleStartedAtMs) {
    static SonarMedian3 filters[3];
    const auto f=filters[0].push(front,sampleStartedAtMs);
    const auto l=filters[1].push(left,sampleStartedAtMs);
    const auto r=filters[2].push(right,sampleStartedAtMs);
    portENTER_CRITICAL(&lateralMux);
    lateralSnapshot.hasFilters=true;
    lateralSnapshot.filteredFront=f;lateralSnapshot.filteredLeft=l;lateralSnapshot.filteredRight=r;
    lateralSnapshot.front = front;
    lateralSnapshot.left = left;
    lateralSnapshot.right = right;
    lateralSnapshot.sampledAtMs = sampleStartedAtMs;
    lateralSnapshot.ready = true;
    portEXIT_CRITICAL(&lateralMux);
}

void waitSonarQuietInterval() {
    const uint32_t started = millis();
    while (static_cast<uint32_t>(millis() - started) < SonarSampling::QUIET_INTERVAL_MS)
        vTaskDelay(1);
}

// Executa no núcleo de sensores; pulseIn() nunca bloqueia o ciclo PID principal.
void lateralSensorTask(void*) {
    for (;;) {
        if (!samplingEnabled.load()) {
            digitalWrite(BoardPins::ULTRASONIC_FRONT.trigger, LOW);
            digitalWrite(BoardPins::ULTRASONIC_LEFT.trigger, LOW);
            digitalWrite(BoardPins::ULTRASONIC_RIGHT.trigger, LOW);
            samplingPaused.store(true);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        samplingPaused.store(false);
        const uint32_t sampleStartedAtMs = millis();
        const UltrasonicReading front = sensorFront.measure();
        waitSonarQuietInterval();
        const UltrasonicReading left = sensorLeft.measure();
        waitSonarQuietInterval();
        const UltrasonicReading right = sensorRight.measure();
        publishSensorSnapshot(front, left, right, sampleStartedAtMs);
        waitSonarQuietInterval();
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

void finishRoute(mm::EndReason reason) {
    if (routeRecorder.running() && routeRecorder.end(millis(),reason))
        telemetryClient.publishSessionEnd(routeBuffer);
}

void safeStop(const char *reason, mm::EndReason endReason = mm::EndReason::Error) {
    snprintf(lastStopReason,sizeof(lastStopReason),"%s",reason);
    terminalTelemetryPublished = false;
    remoteMissionActive = false;
    remoteState = endReason == mm::EndReason::ManualStop ? "stopped" : "error";

    ready = false;
    startDelay.cancel();
    frontEchoRecovery.reset();frontProximity.reset();
    corridorFeedbackFilter.reset();
    motion.stop();
    motors.brake();
    motors.setEnabled(false);
    finishRoute(endReason);
    Serial.printf("Tentativa interrompida: %s\n", reason);
}

void handleMissionUpdate(const nav::MissionUpdate &update) {
    bool started = false;
    if (update.action != nav::MissionAction::None && update.action != nav::MissionAction::Brake) {
        motion.setFrontApproachMode(false);
        automaticFrontReferenceUsed=false;
        motors.setEnabled(true);
    }
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
            remoteMissionActive = false;
            remoteState = update.state == nav::MissionState::Completed ? "completed" : "error";
            if(update.state!=nav::MissionState::Completed && !lastStopReason[0])
                snprintf(lastStopReason,sizeof(lastStopReason),"falha de navegacao: status=%u",static_cast<unsigned>(update.navigationStatus));
            finishRoute(update.state == nav::MissionState::Completed ? mm::EndReason::FinishLine : mm::EndReason::Error);
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
    frontEchoRecovery.reset();frontProximity.reset();
    gridOdometry.beginMotionSegment();
    motionResultPending = true;
    lastControlUpdateUs = micros();
    // O sampler lê os três sensores continuamente; o PID e a HU-01 consomem o mesmo snapshot.
}

void updateWallAlignmentFeedback() {
    // Turns and the explicit front-wall approach do not steer from side echoes.
    if (motion.getMotionType() != MotionType::ForwardCell || motion.isFrontApproachMode()) {
        corridorFeedbackFilter.reset();
        motion.setWallAlignmentFeedback(0, 0, false);
        return;
    }
    const LateralSnapshot snapshot = getLateralSnapshot();
    const auto result = corridorFeedbackFilter.update(
        snapshot.left.distanceCm * 10.0f, snapshot.right.distanceCm * 10.0f,
        snapshot.ready && snapshot.left.valid && snapshot.right.valid,
        snapshot.sampledAtMs, millis());
    if (result.changed)
        motion.setWallAlignmentFeedback(result.leftMm, result.rightMm, result.valid);
}

uint32_t generateTelemetrySessionId() {
    uint32_t id = static_cast<uint32_t>(esp_random() ^ micros() ^ millis());
    if (id == 0) id = 1;
    return id;
}

const char* mazeFormatName() {
    if (mazeConfig.cols == 4 && mazeConfig.rows == 4) return "4x4";
    if (mazeConfig.cols == 8 && mazeConfig.rows == 4) return "8x4";
    if (mazeConfig.cols == 12 && mazeConfig.rows == 4) return "12x4";
    return "desconhecido";
}

const char* telemetryRobotState() {
    if(motion.isSensorPaused())return "STOPPED";
    if(!strcmp(remoteState,"stopped") || !strcmp(remoteState,"armed"))return "STOPPED";
    if(!strcmp(remoteState,"error"))return "ERROR";
    switch (mission.state()) {
        case nav::MissionState::AwaitingObservation:
            return "MAPPING";
        case nav::MissionState::Turning:
        case nav::MissionState::Moving:
            return "NAVIGATING";
        case nav::MissionState::Completed:
            return "FINISHED";
        case nav::MissionState::Failed:
            return "ERROR";
        case nav::MissionState::NotStarted:
        default:
            return "STOPPED";
    }
}

void formatNullableFloat(bool valid, float value, char* output, size_t capacity,
                         unsigned decimals = 2) {
    if (!valid || !isfinite(value)) {
        snprintf(output, capacity, "null");
        return;
    }
    snprintf(output, capacity, decimals == 3 ? "%.3f" : "%.2f", value);
}

void samplePowerMonitor(uint32_t nowMs, bool force = false) {
    if (!powerMonitorReady) {
        powerReadingValid = false;
        return;
    }
    if (!force && static_cast<uint32_t>(nowMs - lastPowerSampleMs) <
                      mm::network::POWER_SAMPLE_PERIOD_MS) {
        return;
    }

    lastPowerSampleMs = nowMs;
    INA226Reading reading{};
    powerReadingValid = powerMonitor.read(reading) &&
                        isfinite(reading.busVoltageV) && isfinite(reading.currentA);
    if (powerReadingValid) latestPowerReading = reading;
}

void publishTelemetryIfDue(uint32_t nowMs) {
    if (!telemetryClientReady || telemetrySessionId == 0) return;
    const nav::MissionState currentMissionState = mission.state();
    const bool terminalState = currentMissionState == nav::MissionState::Completed ||
                               currentMissionState == nav::MissionState::Failed;
    if (terminalState && terminalTelemetryPublished) return;
    if (lastTelemetryPublishMs != 0 &&
        static_cast<uint32_t>(nowMs - lastTelemetryPublishMs) <
            mm::network::TELEMETRY_PERIOD_MS) {
        return;
    }
    lastTelemetryPublishMs = nowMs;
    if (terminalState && !telemetryClient.connected()) return;

    const LateralSnapshot snapshot = getLateralSnapshot();
    const uint32_t sensorAge = snapshot.ready
        ? static_cast<uint32_t>(nowMs - snapshot.sampledAtMs)
        : UINT32_MAX;
    const bool snapshotFresh = snapshot.ready && sensorAge <= MAX_LATERAL_SAMPLE_AGE_MS;

    const auto filteredFront=snapshot.frontEstimate(nowMs);
    const auto filteredLeft=snapshot.leftEstimate(nowMs);
    const auto filteredRight=snapshot.rightEstimate(nowMs);
    char batteryVoltage[20];
    char batteryCurrent[20];
    char frontDistance[20];
    char leftDistance[20];
    char rightDistance[20];
    formatNullableFloat(powerReadingValid, latestPowerReading.busVoltageV,
                        batteryVoltage, sizeof(batteryVoltage), 2);
    formatNullableFloat(powerReadingValid, latestPowerReading.currentA,
                        batteryCurrent, sizeof(batteryCurrent), 3);
    formatNullableFloat(snapshotFresh && filteredFront.valid,
                        filteredFront.distanceCm * 10.0f,
                        frontDistance, sizeof(frontDistance), 2);
    formatNullableFloat(snapshotFresh && filteredLeft.valid,
                        filteredLeft.distanceCm * 10.0f,
                        leftDistance, sizeof(leftDistance), 2);
    formatNullableFloat(snapshotFresh && filteredRight.valid,
                        filteredRight.distanceCm * 10.0f,
                        rightDistance, sizeof(rightDistance), 2);

    const Pose2D pose = motion.getPose();
    float headingDegrees = pose.headingRad * 180.0f / 3.14159265358979323846f;
    while (headingDegrees > 180.0f) headingDegrees -= 360.0f;
    while (headingDegrees < -180.0f) headingDegrees += 360.0f;
    if (!isfinite(headingDegrees)) headingDegrees = 0.0f;

    // Os controladores operam em mm/s; o dashboard apresenta velocidades em m/s.
    const bool motionActive = motion.getStatus() == MotionStatus::Running;
    const float leftSpeedMps = motionActive ? motion.getLeftSpeedMmS() / 1000.0f : 0.0f;
    const float rightSpeedMps = motionActive ? motion.getRightSpeedMmS() / 1000.0f : 0.0f;
    const float pidErrorMps = motionActive ? motion.getPidErrorMmS() / 1000.0f : 0.0f;

    char payload[440];
    const int length = snprintf(
        payload, sizeof(payload),
        "{\"type\":\"telemetry\",\"session_id\":%lu,"
        "\"t_ms\":%lu,\"battery_voltage\":%s,"
        "\"battery_current_a\":%s,\"pid_error\":%.5f,"
        "\"wheels_speed\":{\"left\":%.5f,\"right\":%.5f},"
        "\"heading_deg\":%.2f,\"robot_state\":\"%s\","
        "\"sensor_distances_mm\":{\"front\":%s,"
        "\"left\":%s,\"right\":%s}}",
        static_cast<unsigned long>(telemetrySessionId),
        static_cast<unsigned long>(static_cast<uint32_t>(nowMs - telemetrySessionStartedMs)),
        batteryVoltage, batteryCurrent,
        isfinite(pidErrorMps) ? pidErrorMps : 0.0f,
        isfinite(leftSpeedMps) ? leftSpeedMps : 0.0f,
        isfinite(rightSpeedMps) ? rightSpeedMps : 0.0f,
        headingDegrees, telemetryRobotState(),
        frontDistance, leftDistance, rightDistance);

    if (length <= 0 || static_cast<size_t>(length) >= sizeof(payload)) {
        Serial.println("HU-10: falha ao serializar a telemetria; amostra descartada.");
        return;
    }
    if (!telemetryClient.publishTelemetry(payload, static_cast<size_t>(length))) {
        Serial.printf("HU-10: fila de telemetria indisponível/cheia (descartes=%lu).\n",
                      static_cast<unsigned long>(telemetryClient.droppedMessages()));
    } else if (terminalState) {
        terminalTelemetryPublished = true;
    }
}

bool initializeTelemetry() {
    if (MM_BT_DEVICE_NAME[0] == '\0') {
        Serial.println("HU-10: configure MM_BT_DEVICE_NAME em network_config.local.hpp.");
        return false;
    }

    if (!telemetryClient.begin()) {
        Serial.println("HU-10: não foi possível iniciar o cliente de telemetria.");
        return false;
    }

    telemetrySessionId = generateTelemetrySessionId();
    telemetrySessionStartedMs = millis();
    char sessionStart[160];
    const int length = snprintf(
        sessionStart, sizeof(sessionStart),
        "{\"type\":\"session_start\",\"device_session_id\":%lu,"
        "\"tipo_labirinto\":\"%s\",\"tentativa\":1}",
        static_cast<unsigned long>(telemetrySessionId), mazeFormatName());
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(sessionStart) ||
        !telemetryClient.publishSessionStart(sessionStart, static_cast<size_t>(length))) {
        telemetrySessionId = 0;
        Serial.println("HU-10: falha ao preparar o início da sessão; telemetria desativada.");
        return false;
    }

    terminalTelemetryPublished = false;
    lastTelemetryPublishMs = 0;
    telemetryClientReady = true;
    routeRecorder.begin(telemetrySessionId, robotStateEstimator.currentState().pose,
                        telemetrySessionStartedMs, mazeFormatName(), 1);
    char configPacket[80];
    const int configLength=snprintf(configPacket,sizeof(configPacket),
        "{\"type\":\"maze\",\"width\":%u,\"height\":%u}",mazeConfig.cols,mazeConfig.rows);
    telemetryClient.publishMazeConfig(configPacket,configLength,mazeConfig.cols,mazeConfig.rows);
    Serial.printf("HU-10: cliente Bluetooth iniciado; sessão do dispositivo=%lu; envio nominal a cada %lu ms.\n",
                  static_cast<unsigned long>(telemetrySessionId),
                  static_cast<unsigned long>(mm::network::TELEMETRY_PERIOD_MS));
    return true;
}

bool prepareAttempt(char command, bool fromWebsite = false) {
    const bool corner = command == 'c';
    if (!samplingEnabled.load() || !hardwareReady || ready || startDelay.pending() || (corner ? cornerContinuationUsed : (attemptUsed && !fromWebsite))) {
        Serial.println("INICIO RECUSADO: inicializacao falhou ou tentativa ja utilizada; reinicie.");
        return false;
    }
    if (!telemetryClient.resetMissionCache()) {
        Serial.println("INICIO RECUSADO: aguardando envio da tentativa anterior.");
        return false;
    }
    motors.setEnabled(false);
    Serial.println("Calibrando IMU antes da partida; manter robo imovel.");
    if (!imu.calibrateGyroZ(1000,3) || !encoderLeft.resetCount() || !encoderRight.resetCount()) {
        safeStop("calibracao/encoders na partida"); hardwareReady = false; return false;
    }
    const Pose2D initialPose = {0, 0, initialHeadingRadians(mazeConfig.initialHeading)};
    auto trialConfig = motionConfig;
    if (command != 'g') {
        trialConfig.maxLinearSpeedMmS = 60.0f;
        trialConfig.linearAccelerationMmS2 = 120.0f;
        trialConfig.maxAngularSpeedDegS = 35.0f;
        trialConfig.turnKpDegPerSecPerDeg = 2.0f;
    }
    if (!motion.begin(trialConfig, initialPose) ||
        !mission.begin(mazeConfig, gridOrientation.read().yawDegrees)) {
        safeStop("inicializacao da tentativa"); hardwareReady = false; return false;
    }
    attemptUsed = true;
    motionResultPending = false;
    cellObservation.begin(millis());
    attemptStartedMs = millis();
    lastProcessedSensorTimestampMs = 0;
    guidedRouteActive = command != 'g';
    guidedFrontApproach = false;
    if (corner) { guidedRoute.resetAtCorner(); cornerContinuationUsed = true; }
    else guidedRoute.reset();
    guidedPauseStartedMs = millis();
    if (!guidedRouteActive && !initializeTelemetry()) {
        safeStop("inicializacao da telemetria"); return false;
    }
    remoteMissionActive = fromWebsite;
    lastStopReason[0]=0;peakSensorAgeMs=0;frontEchoRecoveries=0;
    remoteState = guidedRouteActive ? "running" : "armed";
    ready = guidedRouteActive;
    if (!guidedRouteActive) startDelay.arm(millis(), RunConfig::START_DELAY_MS);
    Serial.println(corner
        ? "CONTINUACAO ARMADA: aproximar3cm,esquerda90 a35graus/s,18cm; pausa1s; x interrompe."
        : (guidedRouteActive
            ? "TESTE GUIADO ARMADO: 2x18cm a6cm/s,aproximar3cm a2cm/s,esquerda90 a35graus/s,18cm; pausa1s entre etapas; x interrompe."
            : "MISSAO ARMADA: partida em5s; motores OFF durante espera; x cancela."));
    return true;
}

void publishCurrentMapCell() {
    if (!telemetryClientReady || guidedRouteActive) return;
    char packet[400];
    const int length=mazeMap.cellPacketJson(mission.x(),mission.y(),mission.heading(),packet,sizeof(packet));
    if(length>0) telemetryClient.publishMapSnapshot(mission.x(),mission.y(),packet,length);
    routeRecorder.onTracking(robotStateEstimator.currentState().pose);
}

void publishRobotStatus(uint32_t now, bool force = false) {
    if(!force && now-lastRobotStatusMs<500)return;
    lastRobotStatusMs=now;
    const auto frame=getLateralSnapshot();
    const uint32_t age=frame.ready ? now-frame.sampledAtMs : UINT32_MAX;
    char packet[480];
    const int length=snprintf(packet,sizeof(packet),
        "{\"type\":\"robot_status\",\"firmware\":\"maze\",\"boot_id\":%lu,"
        "\"remote_enabled\":%s,\"state\":\"%s\",\"width\":%u,\"corner\":%u,\"heading\":%u,\"error_reason\":\"%s\",\"sensor_age_ms\":%lu,\"peak_sensor_age_ms\":%lu,\"recovering_front_echo\":%s,\"front_echo_recoveries\":%u}",
        static_cast<unsigned long>(remoteBootId),
        MM_REMOTE_MOTION_ENABLED && hardwareReady ? "true" : "false",
        hardwareReady ? remoteState : "error",mazeConfig.cols,
        static_cast<unsigned>(mazeConfig.startCorner),static_cast<unsigned>(mazeConfig.initialHeading),
        lastStopReason,static_cast<unsigned long>(age),static_cast<unsigned long>(peakSensorAgeMs),
        motion.isSensorPaused()?"true":"false",static_cast<unsigned>(frontEchoRecoveries));
    if(length>0 && length<static_cast<int>(sizeof(packet)))telemetryClient.publishJson(packet,length);
}

void acknowledgeRemote(const char* id, bool accepted, const char* message) {
    JsonDocument ack;
    ack["type"]="robot_ack";ack["id"]=id;ack["accepted"]=accepted;ack["message"]=message;
    char packet[300];const size_t length=serializeJson(ack,packet,sizeof(packet));
    if(length<sizeof(packet))telemetryClient.publishJson(packet,length);
}

void processRemoteLine(const char* line) {
    RemoteCommand command;
    if(!parseRemoteCommand(line,command))return;
    const char* id=command.id;
    if(command.action==RemoteAction::Stop) {
        safeStop("parada pelo site",mm::EndReason::ManualStop);
        motionResultPending=false;
        acknowledgeRemote(id,true,"Motores desabilitados");
        publishRobotStatus(millis(),true);return;
    }
    if(!MM_REMOTE_MOTION_ENABLED) {acknowledgeRemote(id,false,"Partida bloqueada: validar alimentacao e ECHO dos sensores");return;}
    if(command.bootId!=remoteBootId) {
        acknowledgeRemote(id,false,"ESP reiniciou; atualize o estado antes da partida");return;
    }
    if(!strcmp(id,previousStartId) || ready || startDelay.pending() || !hardwareReady) {
        acknowledgeRemote(id,false,"Partida repetida, robo ocupado ou hardware indisponivel");return;
    }
    const int width=command.width,corner=command.corner,heading=command.heading;
    const auto frame=getLateralSnapshot();
    if(!samplingEnabled.load() || !frame.ready || millis()-frame.sampledAtMs>MAX_LATERAL_SAMPLE_AGE_MS ||
       !frame.frontEstimate(millis()).valid || !frame.leftEstimate(millis()).valid || !frame.rightEstimate(millis()).valid) {
        acknowledgeRemote(id,false,"Confira os tres sensores antes de iniciar");return;
    }
    const MazeConfig oldConfig=mazeConfig;
    mazeConfig=makeMazeConfig(width==4?MazeFormat::Maze4x4:width==8?MazeFormat::Maze8x4:MazeFormat::Maze12x4,
        static_cast<MazeCorner>(corner),static_cast<Direction>(heading));
    // Consume the ID even when preparation fails: no automatic start replay.
    strncpy(previousStartId,id,sizeof(previousStartId)-1);
    const bool started=prepareAttempt('g',true);
    if(!started)mazeConfig=oldConfig;
    acknowledgeRemote(id,started,started?"Missao armada; partida em 5 segundos":"Preparacao recusada; confira o robo");
    publishRobotStatus(millis(),true);
}

void handleRemoteCommands() {
    if(!telemetryClient.connected()) {
        remoteWasConnected=false;
        for(int i=0;i<256 && telemetryClient.readCommandByte()>=0;++i){}
        remoteInputLength=0;droppingRemoteInput=false;
        if(remoteMissionActive){safeStop("Bluetooth desconectado");motionResultPending=false;}
        return;
    }
    if(!remoteWasConnected) {
        remoteWasConnected=true;
        // A new link gets a new nonce: old buffered starts cannot run after reconnect.
        remoteBootId=generateTelemetrySessionId();
        publishRobotStatus(millis(),true);
        return;
    }
    for(int count=0;count<256;++count) {
        const int value=telemetryClient.readCommandByte();if(value<0)break;
        const char c=static_cast<char>(value);
        if(c=='\r')continue;
        if(c=='\n') {
            remoteInput[remoteInputLength]=0;
            if(!droppingRemoteInput)processRemoteLine(remoteInput);
            remoteInputLength=0;droppingRemoteInput=false;
        } else if(!droppingRemoteInput) {
            if(remoteInputLength+1<sizeof(remoteInput))remoteInput[remoteInputLength++]=c;
            else {remoteInputLength=0;droppingRemoteInput=true;}
        }
    }
}

void handleSerialCommands() {
    while (Serial.available()) {
        const char command = Serial.read();
        if (command == 'x') { safeStop("parada serial",mm::EndReason::ManualStop); continue; }
        if (command == 'h' || command == '?') {
            Serial.println("g=missao autonoma em5s; t=2F/aproximar/esquerda/F; c=aproximar/esquerda/F do ponto atual; x=parar; s=sensores; d=ecos brutos; p=pausar ultrassons; r=retomar sensores; i=teste isolado. Para outra tentativa, reinicie e reposicione.");
        } else if (command == 'p') {
            safeStop("diagnostico: pausar ultrassons");
            samplingEnabled.store(false);
            const uint32_t requestedAt = millis();
            while (!samplingPaused.load() && millis() - requestedAt < 200) delay(1);
            Serial.println(samplingPaused.load()
                ? "ULTRASSONS PAUSADOS; TRIGs LOW; motores OFF. r retoma apenas sensores."
                : "PAUSA SOLICITADA; ainda sem confirmacao da task.");
        } else if (command == 'i') {
            safeStop("diagnostico isolado dos ultrassons");
            samplingEnabled.store(false);
            const uint32_t requestedAt = millis();
            while (!samplingPaused.load() && millis() - requestedAt < 200) delay(1);
            if (!samplingPaused.load()) { Serial.println("DIAG RECUSADO: task nao confirmou pausa."); continue; }
            Serial.println("DIAGNOSTICO ISOLADO: nao mover robo; motores OFF; 10 leituras por sensor.");
            UltrasonicUS100* sensors[] = {&sensorFront, &sensorLeft, &sensorRight};
            const char* labels[] = {"FRENTE", "ESQUERDA", "DIREITA"};
            for (int side = 0; side < 3; ++side) {
                float values[10]{};
                int echoes = 0, accepted = 0;
                for (int sample = 0; sample < 10; ++sample) {
                    delay(80); // quiet interval, with background triggers paused
                    const auto reading = sensors[side]->measure();
                    if (reading.pulseUs) values[echoes++] = reading.pulseUs / 58.0f;
                    if (reading.valid) ++accepted;
                }
                for (int a = 0; a < echoes; ++a)
                    for (int b = a + 1; b < echoes; ++b)
                        if (values[b] < values[a]) { const float v = values[a]; values[a] = values[b]; values[b] = v; }
                if (echoes) {
                    const float median = echoes % 2 ? values[echoes / 2] : (values[echoes / 2 - 1] + values[echoes / 2]) * 0.5f;
                    Serial.printf("ISOLADO %s ecos=%d/10 aceitos=%d/10 min=%.3f mediana=%.3f max=%.3fcm\n",
                        labels[side], echoes, accepted, values[0], median, values[echoes - 1]);
                    Serial.printf("CORRIGIDO %s mediana=%.3fcm offset=%+.3fcm\n", labels[side],
                                  median + sensors[side]->distanceOffsetCm(), sensors[side]->distanceOffsetCm());
                } else Serial.printf("ISOLADO %s sem eco em10leituras\n", labels[side]);
            }
            Serial.println("DIAG ISOLADO FIM: ultrassons seguem PAUSADOS; motores OFF.");
        } else if (command == 'r') {
            samplingEnabled.store(true);
            Serial.println("ULTRASSONS RETOMADOS; nenhum movimento iniciado.");
        } else if (command == 'd') {
            const auto snapshot = getLateralSnapshot();
            Serial.printf("DIAG amostragem=%s idade=%lums\n", samplingPaused.load() ? "PAUSADA" : "ATIVA",
                          static_cast<unsigned long>(millis() - snapshot.sampledAtMs));
            const UltrasonicReading readings[] = {snapshot.front, snapshot.left, snapshot.right};
            const char* labels[] = {"FRENTE", "ESQUERDA", "DIREITA"};
            for (int i = 0; i < 3; ++i) {
                Serial.printf("%s pulso=%luus bruto=%.2fcm corrigido=%.2fcm estado=%s\n", labels[i],
                    static_cast<unsigned long>(readings[i].pulseUs), readings[i].pulseUs / 58.0f, readings[i].distanceCm,
                    readings[i].valid ? "OK" : (readings[i].tooClose ? "PERTO_DEMAIS" : (readings[i].pulseUs ? "FORA_FAIXA" : "SEM_ECO")));
            }
        } else if (command == 's') {
            const auto snapshot = getLateralSnapshot();
            Serial.printf("SENSORES ready=%u idade=%lums F=%.2f(%u) L=%.2f(%u) R=%.2f(%u) cm; motores=%s\n",
                snapshot.ready, static_cast<unsigned long>(millis() - snapshot.sampledAtMs),
                snapshot.front.distanceCm, snapshot.front.valid, snapshot.left.distanceCm, snapshot.left.valid,
                snapshot.right.distanceCm, snapshot.right.valid, motors.isEnabled() ? "ON" : "OFF");
        } else if (command == 'g' || command == 't' || command == 'c') {
            (void)prepareAttempt(command);
        }
    }
}

}  // namespace

void setup() {
    const bool motorsOK = motors.begin(
        BoardPins::MOTOR_LEFT.in1, BoardPins::MOTOR_LEFT.in2,
        BoardPins::MOTOR_RIGHT.in1, BoardPins::MOTOR_RIGHT.in2,
        BoardPins::MOTOR_NSLEEP, BoardPins::MOTOR_PWM_HZ,
        BoardPins::MOTOR_PWM_BITS);
    Serial.begin(115200);
    remoteBootId=generateTelemetrySessionId();
    telemetryClient.begin();
    delay(100);

    mazeConfig = makeMazeConfig(RunConfig::FORMAT, RunConfig::START, RunConfig::HEADING);
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
    Wire.setTimeOut(20);

    powerMonitorReady = powerMonitor.begin(
        Wire,
        mm::network::INA226_SHUNT_RESISTANCE_OHMS,
        mm::network::INA226_MAX_EXPECTED_CURRENT_A,
        BoardPins::INA226_ADDRESS);
    if (powerMonitorReady) {
        Serial.println("HU-09: INA226 inicializado.");
        samplePowerMonitor(millis(), true);
    } else {
        Serial.println("HU-09: INA226 ausente/não calibrado; bateria será publicada como null.");
    }

    const bool leftEncoderOK = encoderLeft.begin(
        BoardPins::ENCODER_LEFT_A, BoardPins::ENCODER_LEFT_B,
        BoardPins::ENCODER_LEFT_INVERT_DIRECTION);
    const bool rightEncoderOK = encoderRight.begin(
        BoardPins::ENCODER_RIGHT_A, BoardPins::ENCODER_RIGHT_B,
        BoardPins::ENCODER_RIGHT_INVERT_DIRECTION);

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
        BoardPins::ULTRASONIC_FRONT.trigger, BoardPins::ULTRASONIC_FRONT.echo, SonarSampling::ECHO_TIMEOUT_US) &&
        sensorFront.setDistanceOffsetCm(SensorCalibration::FRONT_OFFSET_CM);
    const bool leftSensorOK = sensorLeft.begin(
        BoardPins::ULTRASONIC_LEFT.trigger, BoardPins::ULTRASONIC_LEFT.echo, SonarSampling::ECHO_TIMEOUT_US) &&
        sensorLeft.setDistanceOffsetCm(SensorCalibration::LEFT_OFFSET_CM);
    const bool rightSensorOK = sensorRight.begin(
        BoardPins::ULTRASONIC_RIGHT.trigger, BoardPins::ULTRASONIC_RIGHT.echo, SonarSampling::ECHO_TIMEOUT_US) &&
        sensorRight.setDistanceOffsetCm(SensorCalibration::RIGHT_OFFSET_CM);

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

    // Geometria medida e perfil V17 validado em bancada; integração requer teste no labirinto.
    motionConfig.leftCountsPerMm = MechanicalSettings::LEFT_COUNTS_PER_MM;
    motionConfig.rightCountsPerMm = MechanicalSettings::RIGHT_COUNTS_PER_MM;
    motionConfig.wheelTrackMm = MechanicalSettings::WHEEL_TRACK_MM;
    motionConfig.cellSizeMm = static_cast<float>(mazeConfig.cellSizeMm);
    motionConfig.imuYawSign = 1.0f;

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

    motors.setEnabled(false);
    hardwareReady = true;
    ready = false;
    Serial.printf("Labirinto %ux%u; início=(%u,%u); objetivo=(%u,%u).\n",
                  mazeConfig.rows, mazeConfig.cols,
                  mazeConfig.start.row, mazeConfig.start.col,
                  mazeConfig.goal.row, mazeConfig.goal.col);
    Serial.println("PRONTO E PARADO: g=missao autonoma em5s; t=2F/aproximar/esquerda/F; c=aproximar/esquerda/F do ponto atual; x=parar; s=sensores; h=ajuda. Reiniciar apos terminar.");
    Serial.println("Perfil integrado V17: 100mm/s, 55graus/s; limite 180s. Nenhuma partida automatica.");
}

void loop() {
    handleSerialCommands();
    handleRemoteCommands();
    const uint32_t nowMs = millis();
    const auto ageFrame=getLateralSnapshot();
    if(ageFrame.ready) {
        const uint32_t age=nowMs-ageFrame.sampledAtMs;
        if(age>peakSensorAgeMs)peakSensorAgeMs=age;
    }
    publishRobotStatus(nowMs);
    samplePowerMonitor(nowMs);
    publishTelemetryIfDue(nowMs);

    if (startDelay.consumeIfDue(nowMs)) {
        attemptStartedMs = nowMs;
        cellObservation.begin(millis());
        ready = true;
        remoteState = "running";
        Serial.println("TENTATIVA INICIADA; limite180s a partir de agora; x interrompe.");
    }
    if (!ready) {
        delay(2);
        return;
    }

    if (static_cast<uint32_t>(nowMs - attemptStartedMs) >=
        mazeConfig.attemptTimeoutMs) {
        mission.fail();
        safeStop("tempo máximo da tentativa excedido");
        return;
    }

    const LateralSnapshot sensorSnapshot = getLateralSnapshot();
    if (sensorSnapshot.ready) {
        if (sensorSnapshot.sampledAtMs != lastProcessedSensorTimestampMs) {
            lastProcessedSensorTimestampMs = sensorSnapshot.sampledAtMs;
            robotStateEstimator.updateWalls();
            if (mission.state() == nav::MissionState::AwaitingObservation) {
                const auto& measured=wallPerception.currentState();
                cellObservation.update(measured.timestampMs,
                    measured.frontValid&&measured.leftValid&&measured.rightValid,millis());
            }
        }
    }

    if (motionResultPending) {
        MotionStatus status = motion.getStatus();
        if (status == MotionStatus::Running) {
            const auto snapshot = getLateralSnapshot();
            const bool stale = !snapshot.ready || millis() - snapshot.sampledAtMs > MAX_LATERAL_SAMPLE_AGE_MS;
            const auto filteredFront=snapshot.frontEstimate(millis());
            const bool automaticForward=!guidedRouteActive && motion.isMovingForward();
            if(automaticForward && !stale && (FrontApproach::shouldSlow(filteredFront.valid,filteredFront.distanceCm)||FrontApproach::shouldSlow(snapshot.front.valid,snapshot.front.distanceCm)) &&
               !motion.isFrontApproachMode()) {
                motion.setFrontApproachMode(true);
                Serial.printf("APROXIMACAO AUTONOMA: frente=%.2fcm; velocidade limitada a20mm/s\n",snapshot.front.distanceCm);
            }
            const bool automaticSlow=automaticForward && motion.isFrontApproachMode() &&
                FrontApproach::slowEnough(motion.getLeftSpeedMmS(),motion.getRightSpeedMmS());
            const float frontEmergencyCm = guidedFrontApproach || automaticSlow ? FrontApproach::EMERGENCY_CM : 5.0f;
            const bool rawNear=motion.isMovingForward() &&
                (snapshot.front.tooClose || (snapshot.front.valid && snapshot.front.distanceCm<=frontEmergencyCm));
            const auto proximity=motion.isMovingForward()?frontProximity.update(rawNear,snapshot.sampledAtMs):FrontProximityGuard::State::Clear;
            const bool frontInvalid=motion.isMovingForward() &&
                (!snapshot.front.valid || !isfinite(snapshot.front.distanceCm) || !snapshot.front.pulseUs ||
                 !filteredFront.valid || proximity==FrontProximityGuard::State::Suspect);
            const bool frontBlocked=proximity==FrontProximityGuard::State::Confirmed;
            if (stale || frontBlocked) {
                mission.fail();
                char cause[128];
                if(stale)
                    snprintf(cause,sizeof(cause),"sensores desatualizados: idade=%lums limite=%lums",
                        static_cast<unsigned long>(millis()-snapshot.sampledAtMs),
                        static_cast<unsigned long>(MAX_LATERAL_SAMPLE_AGE_MS));
                else if(!snapshot.front.valid)
                    snprintf(cause,sizeof(cause),"frente invalida: pulso=%luus distancia=%.2fcm perto=%u",
                        static_cast<unsigned long>(snapshot.front.pulseUs),snapshot.front.distanceCm,snapshot.front.tooClose);
                else
                    snprintf(cause,sizeof(cause),"reserva frontal confirmada: bruto=%.2fcm limite=%.2fcm pulso=%luus",
                        snapshot.front.distanceCm,frontEmergencyCm,static_cast<unsigned long>(snapshot.front.pulseUs));
                safeStop(cause);
                motionResultPending = false;
                return;
            }
            if(motion.isMovingForward()) {
                const auto decision=frontEchoRecovery.update(!frontInvalid,snapshot.sampledAtMs,millis(),
                    motion.sensorPauseSettled());
                if(decision==FrontEchoRecovery::Decision::Failed) {
                    mission.fail();
                    safeStop("eco frontal instavel: recuperacao excedeu 1200ms ou 3 pausas no trecho");
                    motionResultPending=false;return;
                }
                const bool hold=decision==FrontEchoRecovery::Decision::Hold;
                const bool wasPaused=motion.isSensorPaused();
                if(!motion.setSensorPaused(hold)) {
                    mission.fail();safeStop("falha ao pausar/retomar apos perda do eco frontal");
                    motionResultPending=false;return;
                }
                if(hold) {
                    corridorFeedbackFilter.reset();
                    motion.setWallAlignmentFeedback(0,0,false);
                }
                if(hold!=wasPaused) {
                    if(hold && frontEchoRecoveries<65535)++frontEchoRecoveries;
                    Serial.println(hold ? "ECO FRONTAL: freando e aguardando novas leituras" : "ECO FRONTAL: duas leituras validas; retomando o restante do trecho");
                    publishRobotStatus(millis(),true);
                }
            }
            if(automaticSlow && !motion.isSensorPaused() && filteredFront.valid &&
               filteredFront.distanceCm>FrontApproach::TARGET_CM+FrontApproach::TOLERANCE_CM &&
               filteredFront.distanceCm<=5.0f &&
               FrontApproach::positionMatchesCell(motion.getDistanceTravelledMm(),mazeConfig.cellSizeMm)) {
                // A brake in progress may finish; geometry is checked again afterwards.
                if(motion.refineFrontApproachTarget(filteredFront.distanceCm))automaticFrontReferenceUsed=true;
            }
            if(automaticForward && motion.isFrontApproachMode() && !motion.isSensorPaused() &&
               FrontApproach::atTarget(filteredFront.valid,filteredFront.distanceCm)) {
                if(!FrontApproach::positionMatchesCell(motion.getDistanceTravelledMm(),mazeConfig.cellSizeMm)) {
                    char cause[128];
                    snprintf(cause,sizeof(cause),"parede antes do centro: eixo avancou %.1fmm; celula=%umm; frente=%.2fcm",
                        motion.getDistanceTravelledMm(),mazeConfig.cellSizeMm,filteredFront.distanceCm);
                    mission.fail();safeStop(cause);motionResultPending=false;return;
                }
                automaticFrontReferenceUsed=true;
                motion.brakeAtFrontTarget();
            }
            if (!motion.isSensorPaused() && guidedFrontApproach && filteredFront.valid &&
                filteredFront.distanceCm <= MazeSettings::GUIDED_FRONT_TARGET_CM + MazeSettings::GUIDED_FRONT_TOLERANCE_CM)
                motion.brakeAtFrontTarget();
            const uint32_t nowUs = micros();
            const uint32_t elapsedUs = static_cast<uint32_t>(nowUs - lastControlUpdateUs);
            if (elapsedUs >= CONTROL_PERIOD_US) {
                motion.update(static_cast<float>(elapsedUs) / 1000000.0f);
                // Confirm grid turns only after settling. Updating at every
                // intermediate 75-degree threshold would accumulate yaw-reference
                // errors across consecutive turns in PositionTracker.
                if (motion.getMotionType() == MotionType::ForwardCell ||
                    motion.getStatus() != MotionStatus::Running)
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
        Serial.printf("MOVIMENTO status=%u motivo=%u distancia=%.2fmm yaw=%.2fgraus\n",
            static_cast<unsigned>(status), static_cast<unsigned>(motion.getFailureReason()),
            motion.getDistanceTravelledMm(), motion.getTurnAngleDeg());
        if (guidedRouteActive) {
            if (guidedFrontApproach && status == MotionStatus::Succeeded) {
                const auto snapshot = getLateralSnapshot();
                const bool atTarget = snapshot.ready && snapshot.front.valid &&
                    millis() - snapshot.sampledAtMs <= MAX_LATERAL_SAMPLE_AGE_MS &&
                    fabsf(snapshot.front.distanceCm - MazeSettings::GUIDED_FRONT_TARGET_CM) <= MazeSettings::GUIDED_FRONT_TOLERANCE_CM;
                if (!atTarget) { safeStop("aproximacao frontal terminou fora da meta de3cm"); return; }
                Serial.printf("APROXIMACAO CONFIRMADA: frente=%.2fcm\n", snapshot.front.distanceCm);
            }
            guidedFrontApproach = false;
            motion.setFrontApproachMode(false);
            guidedRoute.finishMovement(status == MotionStatus::Succeeded);
            guidedPauseStartedMs = millis();
            if (guidedRoute.action() == nav::GuidedAction::Failed) {
                safeStop("falha no teste guiado");
            } else if (guidedRoute.action() == nav::GuidedAction::Complete) {
                ready = false;
                guidedRouteActive = false;
                motors.setEnabled(false);
                Serial.println("TESTE GUIADO CONCLUIDO; motores OFF. Reinicie/reposicione antes de outra tentativa.");
            }
            return;
        }
        if(status==MotionStatus::Succeeded && motion.isMovingForward()) {
            const auto frame=getLateralSnapshot();
            const bool fresh=frame.ready && millis()-frame.sampledAtMs<=MAX_LATERAL_SAMPLE_AGE_MS;
            if(!FrontApproach::arrivalConfirmed(automaticFrontReferenceUsed,fresh&&frame.frontEstimate(millis()).valid,frame.frontEstimate(millis()).distanceCm,
                motion.getDistanceTravelledMm(),mazeConfig.cellSizeMm)) {
                char cause[128];
                snprintf(cause,sizeof(cause),"chegada nao confirmada: avanço=%.1fmm frente=%.2fcm; celula=%umm refFrontal=%u",
                    motion.getDistanceTravelledMm(),frame.front.distanceCm,mazeConfig.cellSizeMm,automaticFrontReferenceUsed);
                mission.fail();safeStop(cause);return;
            }
            Serial.printf("CELULA CONFIRMADA: avanço=%.1fmm frente=%.2fcm; observar antes da proxima acao\n",
                motion.getDistanceTravelledMm(),frame.front.distanceCm);
        }
        if(status!=MotionStatus::Succeeded)
            snprintf(lastStopReason,sizeof(lastStopReason),"movimento falhou: codigo=%u",static_cast<unsigned>(motion.getFailureReason()));
        const nav::MissionUpdate missionUpdate =
            mission.onMotionFinished(status == MotionStatus::Succeeded);
        publishCurrentMapCell();
        handleMissionUpdate(missionUpdate);
        if (missionUpdate.state == nav::MissionState::AwaitingObservation) {
            cellObservation.begin(millis());
        }
        return;
    }

    if (guidedRouteActive) {
        if (millis() - guidedPauseStartedMs < 1000) { delay(1); return; }
        const auto snapshot = getLateralSnapshot();
        if (!snapshot.ready || millis() - snapshot.sampledAtMs > MAX_LATERAL_SAMPLE_AGE_MS) {
            safeStop("sensores desatualizados antes da etapa guiada"); return;
        }
        const bool approach = guidedRoute.action() == nav::GuidedAction::ApproachFrontWall;
        if (approach) {
            if (!snapshot.front.valid || snapshot.front.distanceCm <= MazeSettings::GUIDED_FRONT_EMERGENCY_CM) {
                safeStop("aproximacao frontal invalida ou sem reserva"); return;
            }
            const float extraMm = (snapshot.front.distanceCm - MazeSettings::GUIDED_FRONT_TARGET_CM) * 10.0f;
            if (fabsf(extraMm) <= MazeSettings::GUIDED_FRONT_TOLERANCE_CM * 10.0f) {
                guidedRoute.finishMovement(true); guidedPauseStartedMs = millis();
                Serial.println("APROXIMACAO: ja na faixa de3cm; sem movimento."); return;
            }
            if (extraMm <= 0 || extraMm > mazeConfig.cellSizeMm) {
                safeStop("parede fora da faixa prevista para aproximacao de esquina"); return;
            }
            guidedFrontApproach = true;
            motion.setFrontApproachMode(true);
            motors.setEnabled(true);
            if (!motion.startMoveLinear(extraMm)) { safeStop("falha ao iniciar aproximacao"); return; }
            frontEchoRecovery.reset();frontProximity.reset();
            gridOdometry.beginMotionSegment();
            motionResultPending = true; lastControlUpdateUs = micros();
            Serial.printf("TESTE ETAPA 3/5: aproximacao lenta, alvo3cm; max20mm/s; extra estimado%.1fmm\n", extraMm);
            return;
        }
        const bool forward = guidedRoute.action() == nav::GuidedAction::ForwardCell;
        if (forward && (!snapshot.front.valid || snapshot.front.distanceCm <= 5.0f)) {
            safeStop("frente invalida ou obstaculo antes da etapa guiada"); return;
        }
        if (forward && guidedRoute.step() < 2) {
            const float segmentMm = nav::guidedPreCornerMoveMm(snapshot.front.distanceCm);
            if (segmentMm <= 1.0f) {
                guidedRoute.resetAtCorner(); guidedPauseStartedMs = millis();
                Serial.println("ESQUINA: transicao antecipada para aproximacao lenta; sem insistir em celula inteira.");
                return;
            }
            motion.setFrontApproachMode(false);
            motors.setEnabled(true);
            if (!motion.startMoveLinear(segmentMm)) { safeStop("falha ao iniciar trecho antes da curva"); return; }
            frontEchoRecovery.reset();frontProximity.reset();
            gridOdometry.beginMotionSegment();
            motionResultPending = true; lastControlUpdateUs = micros();
            Serial.printf("TESTE ETAPA %u/5: avancar ate%.1fmm, preservando8cm antes da aproximacao\n",
                          guidedRoute.step() + 1, segmentMm);
            return;
        }
        if (!forward && (!snapshot.left.valid || snapshot.left.distanceCm <= 5.0f)) {
            safeStop("abertura esquerda nao confirmada antes do giro guiado"); return;
        }
        Serial.printf("TESTE ETAPA %u/5: %s\n", guidedRoute.step() + 1,
                      forward ? "avancar18cm" : "esquerda90graus");
        const nav::MissionUpdate update{nav::MissionState::NotStarted,
            forward ? nav::MissionAction::MoveForwardOneCell : nav::MissionAction::TurnLeft90,
            nav::NavigationStatus::NotInitialized};
        handleMissionUpdate(update);
        return;
    }

    if (mission.state() == nav::MissionState::AwaitingObservation) {
        const auto& state = robotStateEstimator.currentState();
        const uint32_t sampleAge = static_cast<uint32_t>(millis() - state.sensorTimestampMs);
        if(cellObservation.expired(millis())) {
            const auto& measured=wallPerception.currentState();
            char cause[128];
            snprintf(cause,sizeof(cause),"paredes instaveis por1500ms: frente=%u esquerda=%u direita=%u",
                measured.frontValid,measured.leftValid,measured.rightValid);
            mission.fail();safeStop(cause);return;
        }
        if (!state.initialized || !cellObservation.ready(millis()) ||
            sampleAge > MAX_LATERAL_SAMPLE_AGE_MS) {
            delayMicroseconds(500);
            return;
        }
        const auto next = mission.observeCurrentCell();
        publishCurrentMapCell();
        handleMissionUpdate(next);
    }
}
