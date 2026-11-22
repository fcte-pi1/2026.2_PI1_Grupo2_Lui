#include <Arduino.h>
#include <ArduinoJson.h>
#include "communication/RemoteCommand.h"
#include "navigation/SessionRecorder.h"
#include <atomic>
#include <utility>
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
#include "control/guided_lateral_clearance.hpp"
#include "control/guided_wall_recovery.hpp"
#include "hal/SnapshotWallSensor.h"
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
    tracking::OdometryReading read() override {
        return {completedDistanceMm_, millis()};
    }

    void resetAccumulator() override { completedDistanceMm_ = 0; }
    void reportArrival(float distanceMm) { completedDistanceMm_ = static_cast<int16_t>(lroundf(distanceMm)); }

private:
    int16_t completedDistanceMm_ = 0;
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
MotionDistanceReader gridOdometry;
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
bool missionCornerActive = false;
bool missionMovingForward = false;
int cornerTurnDirection = 1;
float missionExitCreditMm = 0.0f;
float missionLinearProgressMm = 0.0f;
bool guidedFrontApproach = false;
bool automaticFrontReferenceUsed=false;
bool cornerContinuationUsed = false;
nav::GuidedRoute guidedRoute;
GuidedLateralClearance guidedLateralClearance;
GuidedWallRecovery guidedRecovery;
uint32_t guidedPauseStartedMs = 0;
float guidedCornerRemainingMm = 0.0f;
float guidedCornerShiftTravelMm = 0.0f;
struct GuidedPivotRecovery {
    enum class Phase { None, BeforeReturn, Returning, BeforeAdjustment, Adjusting };
    Phase phase = Phase::None;
    bool frontBlocked = false;
    bool leftBlocked = false;
    bool rightBlocked = false;
} guidedPivotRecovery;
MotionControllerConfig motionConfig;
struct PivotDiagnostic {
    float targetDeg = 0.0f;
    float peakAxleShiftMm = 0.0f;
    uint32_t startedMs = 0;
    uint32_t lastSampleMs = 0;

    bool active() const { return targetDeg != 0.0f; }
} pivotDiagnostic;
bool motionResultPending = false;
uint32_t attemptStartedMs = 0;
uint32_t lastControlUpdateUs = 0;
uint32_t lastProcessedSensorTimestampMs = 0;
CellObservationGate cellObservation;

bool cornerSequenceActive() { return guidedRouteActive || missionCornerActive; }

LateralSnapshot motionSnapshot() {
    auto frame = getLateralSnapshot();
    if (cornerSequenceActive() && cornerTurnDirection < 0) {
        // As regras do canto usam esquerda=interior e direita=exterior.
        std::swap(frame.left, frame.right);
        std::swap(frame.filteredLeft, frame.filteredRight);
    }
    return frame;
}

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

    if (ready && !guidedRouteActive) mission.fail();
    ready = false;
    missionCornerActive = false;
    missionMovingForward = false;
    missionExitCreditMm = 0.0f;
    startDelay.cancel();
    pivotDiagnostic = {};
    guidedRecovery = {};
    guidedPivotRecovery = {};
    frontEchoRecovery.reset();frontProximity.reset();
    motion.stop();
    motors.brake();
    motors.setEnabled(false);
    finishRoute(endReason);
    Serial.printf("Tentativa interrompida: %s\n", reason);
}

void trackStartedMotion() {
    frontEchoRecovery.reset();
    frontProximity.reset();
    motionResultPending = true;
    lastControlUpdateUs = micros();
}

bool guidedLinearInterruption(MotionFailureReason reason) {
    return reason == MotionFailureReason::Obstacle || reason == MotionFailureReason::LeftWheelStalled ||
        reason == MotionFailureReason::RightWheelStalled || reason == MotionFailureReason::WrongDirection;
}

bool handleGuidedRecoveryResult(MotionStatus status) {
    using PivotPhase = GuidedPivotRecovery::Phase;
    if (guidedPivotRecovery.phase == PivotPhase::BeforeReturn) {
        if (motion.getFailureReason() != MotionFailureReason::Obstacle ||
            fabsf(motion.getDistanceTravelledMm()) > 6.0f) {
            safeStop("curva: posicao de retorno do giro nao confirmada");
        } else {
            if (fabsf(motion.getTurnAngleDeg()) < 1.2f)
                guidedPivotRecovery.phase = PivotPhase::BeforeAdjustment;
            guidedPauseStartedMs = millis();
        }
        return true;
    }
    if (guidedPivotRecovery.phase == PivotPhase::Returning) {
        if (status != MotionStatus::Succeeded) {
            safeStop("curva: nao foi possivel retornar ao rumo da diagonal");
        } else {
            guidedPivotRecovery.phase = PivotPhase::BeforeAdjustment;
            guidedPauseStartedMs = millis();
        }
        return true;
    }
    if (guidedPivotRecovery.phase == PivotPhase::Adjusting) {
        const float travelledMm = motion.getDistanceTravelledMm();
        if (status != MotionStatus::Succeeded || fabsf(travelledMm) < 2.0f) {
            safeStop("curva: ajuste de posicao sem progresso confirmado");
        } else {
            guidedCornerShiftTravelMm += travelledMm;
            guidedPivotRecovery = {};
            guidedRecovery = {};
            motion.setFrontApproachMode(false);
            guidedPauseStartedMs = millis();
            Serial.printf("CURVA: ajuste de %+.1fmm confirmado; repetindo o giro final\n", travelledMm);
        }
        return true;
    }
    using Phase = GuidedWallRecovery::Phase;
    if (guidedRecovery.phase == Phase::Braking) {
        if (!guidedLinearInterruption(motion.getFailureReason())) {
            safeStop("recuperacao guiada: falha ao confirmar parada");
            return true;
        }
        const bool trimDiagonal = cornerSequenceActive() && guidedRoute.action() == nav::GuidedAction::ShiftIntoCorner &&
            guidedRecovery.rightBlocked && !guidedRecovery.leftBlocked && !guidedRecovery.frontBlocked;
        const float retreatLimitMm = trimDiagonal ? nav::guidedCornerRetreatLimitMm(guidedCornerShiftTravelMm) : 30.0f;
        guidedRecovery.retreatDistance(
            motion.getDistanceTravelledMm(), motion.getLinearTargetMm(), retreatLimitMm);
        guidedPauseStartedMs = millis();
        return true;
    }
    if (guidedRecovery.phase == Phase::Adjusting) {
        if (status != MotionStatus::Succeeded && !guidedLinearInterruption(motion.getFailureReason())) {
            safeStop("recuperacao guiada: falha de controle durante o ajuste");
            return true;
        }
        guidedRecovery.finishAdjustment(motion.getDistanceTravelledMm(), status == MotionStatus::Succeeded);
        guidedPauseStartedMs = millis();
        return true;
    }
    return false;
}

void handleMissionUpdate(const nav::MissionUpdate &update) {
    using Action = nav::MissionAction;
    const auto action = update.action;
    if (update.turnTowardNextCell && (action == Action::TurnLeft90 || action == Action::TurnRight90)) {
        const auto snapshot = getLateralSnapshot();
        const uint32_t now = millis();
        const bool leftTurn = action == Action::TurnLeft90;
        const auto& rawExit = leftTurn ? snapshot.left : snapshot.right;
        const auto exit = leftTurn ? snapshot.leftEstimate(now) : snapshot.rightEstimate(now);
        const auto front = snapshot.frontEstimate(now);
        if (snapshot.ready && now - snapshot.sampledAtMs <= MAX_LATERAL_SAMPLE_AGE_MS &&
            snapshot.front.valid && !snapshot.front.tooClose && rawExit.valid && !rawExit.tooClose &&
            front.valid && exit.valid &&
            nav::missionCornerEligible(snapshot.front.distanceCm, rawExit.distanceCm, mazeConfig.cellSizeMm) &&
            nav::missionCornerEligible(front.distanceCm, exit.distanceCm, mazeConfig.cellSizeMm)) {
            missionCornerActive = true;
            missionMovingForward = false;
            cornerTurnDirection = leftTurn ? 1 : -1;
            guidedRoute.resetAtCorner();
            guidedCornerRemainingMm = FrontApproach::POSITION_TOLERANCE_MM;
            guidedCornerShiftTravelMm = 0.0f;
            guidedFrontApproach = false;
            guidedLateralClearance.reset();
            guidedPauseStartedMs = now;
            Serial.printf("MISSAO: curva em duas etapas para %s\n", leftTurn ? "esquerda" : "direita");
            return;
        }
    }
    bool started = false;
    if (action != Action::None && action != Action::Brake) {
        motion.setFrontApproachMode(false);
        automaticFrontReferenceUsed=false;
        missionMovingForward = action == Action::MoveForwardOneCell;
        missionLinearProgressMm = missionMovingForward ? missionExitCreditMm : 0.0f;
        missionExitCreditMm = 0.0f;
        guidedFrontApproach = false;
        if (action == Action::TurnLeft90 || action == Action::TurnRight90 || action == Action::TurnAround180)
            guidedLateralClearance.reset();
        motors.setEnabled(true);
    }
    switch (action) {
        case nav::MissionAction::None:
            return;
        case nav::MissionAction::MoveForwardOneCell:
            started = motion.startMoveLinear(mazeConfig.cellSizeMm - missionLinearProgressMm);
            break;
        case nav::MissionAction::MoveBackwardOneCell:
            started = motion.startMoveBackwardOneCell();
            break;
        case nav::MissionAction::TurnLeft90:
            started = motion.startGuidedPivot(90.0f);
            break;
        case nav::MissionAction::TurnRight90:
            started = motion.startGuidedPivot(-90.0f);
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
    trackStartedMotion();
    // O sampler lê os três sensores continuamente; o PID e a HU-01 consomem o mesmo snapshot.
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

MotionControllerConfig guidedMotionConfig() {
    auto config = motionConfig;
    config.maxLinearSpeedMmS = 60.0f;
    config.linearAccelerationMmS2 = 120.0f;
    config.forwardTimeoutMs = 8500; // Um trecho inteiro pode avançar a 35 mm/s.
    config.maxAngularSpeedDegS = 35.0f;
    config.turnKpDegPerSecPerDeg = 2.0f;
    return config;
}

bool prepareAttempt(char command, bool fromWebsite = false) {
    const bool corner = command == 'c';
    if (!samplingEnabled.load() || !hardwareReady || ready || startDelay.pending() || pivotDiagnostic.active() ||
        (corner ? cornerContinuationUsed : (attemptUsed && !fromWebsite))) {
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
    const auto trialConfig = guidedMotionConfig();
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
    missionCornerActive = false;
    missionMovingForward = false;
    cornerTurnDirection = 1;
    missionExitCreditMm = missionLinearProgressMm = 0.0f;
    guidedFrontApproach = false;
    guidedLateralClearance.reset();
    guidedRecovery = {};
    guidedPivotRecovery = {};
    if (corner) { guidedRoute.resetAtCorner(); cornerContinuationUsed = true; }
    else guidedRoute.reset();
    guidedCornerRemainingMm = corner ? mazeConfig.cellSizeMm :
        3.0f * mazeConfig.cellSizeMm + FrontApproach::POSITION_TOLERANCE_MM;
    guidedCornerShiftTravelMm = 0.0f;
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
        ? "CONTINUACAO ARMADA: aproximar3cm,curva a esquerda em duas partes,18cm; pausa1s; x interrompe."
        : (guidedRouteActive
            ? "TESTE GUIADO ARMADO: 2x18cm a6cm/s,aproximar3cm a2cm/s,curva a esquerda em duas partes,18cm; pausa1s entre etapas; x interrompe."
            : "MISSAO ARMADA: partida em5s; motores OFF durante espera; x cancela."));
    return true;
}

void armPivotDiagnostic(float targetDeg) {
    if (!hardwareReady || !samplingEnabled.load() || ready || startDelay.pending() || pivotDiagnostic.active()) {
        Serial.println("GIRO TESTE RECUSADO: robo ocupado, sensores pausados ou hardware indisponivel.");
        return;
    }
    motors.setEnabled(false);
    Serial.println("GIRO TESTE: calibrando IMU; mantenha o robo imovel e em piso livre.");
    if (!imu.calibrateGyroZ()) {
        safeStop("falha na calibracao da IMU para o giro");
        return;
    }
    pivotDiagnostic = {};
    pivotDiagnostic.targetDeg = targetDeg;
    startDelay.arm(millis(), RunConfig::START_DELAY_MS);
    Serial.printf("GIRO TESTE ARMADO: %.0f graus para %s em 5s; x cancela.\n",
                  fabsf(targetDeg), targetDeg > 0.0f ? "esquerda" : "direita");
}

void updatePivotDiagnostic() {
    const bool starting = startDelay.consumeIfDue(millis());
    if (!starting && startDelay.pending()) return;

    const auto frame = getLateralSnapshot();
    const uint32_t nowMs = millis();
    if (!frame.ready || nowMs - frame.sampledAtMs > MAX_LATERAL_SAMPLE_AGE_MS) {
        safeStop("giro teste: quadro dos sensores desatualizado");
        return;
    }
    // Em piso aberto pode não haver eco. Um eco curto continua protegendo a manobra.
    const auto tooNear = [](const UltrasonicReading& reading) {
        return reading.tooClose || (reading.valid && reading.distanceCm <= 5.0f);
    };
    const bool blocked = tooNear(frame.front) || tooNear(frame.left) || tooNear(frame.right);
    if (starting) {
        if (blocked) { safeStop("giro teste: coloque o robo longe das paredes"); return; }
        imu.resetYaw();
        if (!motion.begin(guidedMotionConfig(), {0.0f, 0.0f, 0.0f})) {
            safeStop("giro teste: falha ao iniciar encoders ou IMU");
            return;
        }
        motors.setEnabled(true);
        if (!motion.startGuidedPivot(pivotDiagnostic.targetDeg)) {
            safeStop("giro teste: falha ao iniciar movimento");
            return;
        }
        const float arcMm = fabsf(pivotDiagnostic.targetDeg) * MechanicalSettings::kPi / 180.0f *
            motionConfig.wheelTrackMm * 0.5f;
        Serial.printf("GIRO TESTE: arco nominal %.1fmm por roda; bitola=%.1fmm; ticks/mm E=%.3f D=%.3f\n",
                      arcMm, motionConfig.wheelTrackMm, motionConfig.leftCountsPerMm, motionConfig.rightCountsPerMm);
        pivotDiagnostic.startedMs = pivotDiagnostic.lastSampleMs = millis();
        lastControlUpdateUs = micros();
        return;
    }

    if (blocked) motion.brakeForObstacle();
    const uint32_t nowUs = micros();
    const uint32_t elapsedUs = nowUs - lastControlUpdateUs;
    if (elapsedUs < CONTROL_PERIOD_US) return;
    lastControlUpdateUs = nowUs;
    motion.update(static_cast<float>(elapsedUs) / 1000000.0f);
    const auto pose = motion.getPose();
    const float shiftMm = hypotf(pose.xMm, pose.yMm);
    pivotDiagnostic.peakAxleShiftMm = fmaxf(pivotDiagnostic.peakAxleShiftMm, shiftMm);

    const auto status = motion.getStatus();
    const bool done = status != MotionStatus::Running;
    if (!done && nowMs - pivotDiagnostic.lastSampleMs < 250) return;
    pivotDiagnostic.lastSampleMs = nowMs;
    const auto travel = motion.getWheelTravel();
    const float encoderDeg = (travel.rightMm - travel.leftMm) / motionConfig.wheelTrackMm *
        180.0f / MechanicalSettings::kPi;
    Serial.printf("GIRO %s ms=%lu IMU=%+.2fgraus encoder=%+.2fgraus E=%+.2fmm(%ld ticks) D=%+.2fmm(%ld ticks) eixo=(%+.2f,%+.2f)mm\n",
                  done ? "FINAL" : "AMOSTRA", static_cast<unsigned long>(nowMs - pivotDiagnostic.startedMs),
                  motion.getTurnAngleDeg(), encoderDeg, travel.leftMm, static_cast<long>(travel.leftTicks),
                  travel.rightMm, static_cast<long>(travel.rightTicks), pose.xMm, pose.yMm);
    if (done) {
        Serial.printf("GIRO RESULTADO status=%u motivo=%u alvo=%+.0fgraus deriva_final=%.2fmm pico=%.2fmm\n",
                      static_cast<unsigned>(status), static_cast<unsigned>(motion.getFailureReason()),
                      pivotDiagnostic.targetDeg, shiftMm, pivotDiagnostic.peakAxleShiftMm);
        pivotDiagnostic = {};
    }
}

void publishCurrentMapCell() {
    if (!telemetryClientReady || guidedRouteActive) return;
    char packet[400];
    const int length=mazeMap.cellPacketJson(mission.x(),mission.y(),mission.heading(),packet,sizeof(packet));
    if(length>0) telemetryClient.publishMapSnapshot(mission.x(),mission.y(),packet,length);
    routeRecorder.onTracking(robotStateEstimator.currentState().pose);
}

void finishMissionMotion(bool succeeded, bool frontExitConfirmed = false) {
    if (succeeded && mission.state() == nav::MissionState::Moving) {
        const auto frame = getLateralSnapshot();
        const uint32_t now = millis();
        const auto front = frame.frontEstimate(now);
        const bool fresh = frame.ready && now - frame.sampledAtMs <= MAX_LATERAL_SAMPLE_AGE_MS;
        const float travelledMm = fabsf(missionLinearProgressMm);
        if ((missionMovingForward && missionLinearProgressMm <= 0.0f) ||
            (!missionMovingForward && missionLinearProgressMm >= 0.0f) ||
            !FrontApproach::arrivalConfirmed(automaticFrontReferenceUsed, fresh && front.valid,
                                              front.distanceCm, travelledMm, mazeConfig.cellSizeMm)) {
            char cause[128];
            snprintf(cause, sizeof(cause), "chegada nao confirmada: percurso=%+.1fmm frente=%.2fcm celula=%umm",
                     missionLinearProgressMm, frame.front.distanceCm, mazeConfig.cellSizeMm);
            safeStop(cause);
            return;
        }
        gridOdometry.reportArrival(missionLinearProgressMm);
        Serial.printf("CELULA CONFIRMADA: percurso=%+.1fmm; aguardando observacao\n", missionLinearProgressMm);
    }
    // A grade recebe uma confirmação por movimento lógico, após o repouso.
    // Recuos, rotações parciais e a diagonal não avançam uma célula por si sós.
    if (succeeded) robotStateEstimator.updatePose();
    gridOdometry.resetAccumulator();
    missionMovingForward = false;
    if (!succeeded)
        snprintf(lastStopReason, sizeof(lastStopReason), "movimento falhou: codigo=%u",
                 static_cast<unsigned>(motion.getFailureReason()));
    const auto update = mission.onMotionFinished(succeeded, frontExitConfirmed);
    publishCurrentMapCell();
    handleMissionUpdate(update);
    if (update.state == nav::MissionState::AwaitingObservation) cellObservation.begin(millis());
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
    if(!strcmp(id,previousStartId) || ready || startDelay.pending() || pivotDiagnostic.active() || !hardwareReady) {
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
            Serial.println("v=teste de giro 90 graus a esquerda; V=90 graus a direita; 5s de espera; piso livre; x cancela.");
        } else if (command == 'v' || command == 'V') {
            armPivotDiagnostic(command == 'v' ? 90.0f : -90.0f);
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
    Serial.println("PRONTO E PARADO: g=missao autonoma; t=trajeto em L; c=continuar da esquina; v/V=teste de giro; x=parar; s=sensores; h=ajuda.");
    Serial.println("Perfil de movimento: 60mm/s, aproximacao a20mm/s, giros ate35graus/s; limite 180s. Nenhuma partida automatica.");
}

void loop() {
    handleSerialCommands();
    handleRemoteCommands();
    if (pivotDiagnostic.active()) {
        updatePivotDiagnostic();
        delayMicroseconds(500);
        return;
    }
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
    if (sensorSnapshot.ready && !missionCornerActive) {
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
        using RecoveryPhase = GuidedWallRecovery::Phase;
        MotionStatus status = motion.getStatus();
        if (status == MotionStatus::Running) {
            const auto snapshot = motionSnapshot();
            const uint32_t sensorNowMs = millis();
            const bool stale = !snapshot.ready || sensorNowMs - snapshot.sampledAtMs > MAX_LATERAL_SAMPLE_AGE_MS;
            const auto filteredFront=snapshot.frontEstimate(sensorNowMs);
            const bool automaticForward = !cornerSequenceActive() && missionMovingForward && motion.isMovingForward() &&
                guidedRecovery.phase != RecoveryPhase::Adjusting && guidedRecovery.phase != RecoveryPhase::Braking;
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
            const bool guidedForward = motion.isMovingForward();
            const bool shifting = guidedForward && cornerSequenceActive() &&
                guidedRoute.action() == nav::GuidedAction::ShiftIntoCorner;
            const bool adjusting = guidedRecovery.phase == RecoveryPhase::Adjusting;
            const bool retreating = adjusting && guidedRecovery.plannedAdjustmentMm < 0.0f;
            const bool validRawFront = snapshot.front.valid && !snapshot.front.tooClose &&
                snapshot.front.pulseUs && isfinite(snapshot.front.distanceCm);
            const bool clearingFrontBlindZone = retreating && guidedRecovery.clearingFrontBlindZone();
            if (clearingFrontBlindZone && validRawFront && filteredFront.valid)
                guidedRecovery.observeFront(fminf(snapshot.front.distanceCm, filteredFront.distanceCm));
            const bool frontReleasing = clearingFrontBlindZone && snapshot.front.pulseUs &&
                (snapshot.front.tooClose || validRawFront);
            const bool retreatFrontInvalid = retreating &&
                !frontReleasing && (!validRawFront || !filteredFront.valid ||
                 !isfinite(filteredFront.distanceCm) ||
                 !guidedRecovery.frontAllowsRetreat(fminf(snapshot.front.distanceCm, filteredFront.distanceCm)));
            const auto filteredLeft = snapshot.leftEstimate(sensorNowMs);
            const auto filteredRight = snapshot.rightEstimate(sensorNowMs);
            const float leftCm = snapshot.left.valid && !snapshot.left.tooClose && filteredLeft.valid
                ? filteredLeft.distanceCm : NAN;
            const float rightCm = snapshot.right.valid && !snapshot.right.tooClose && filteredRight.valid
                ? filteredRight.distanceCm : NAN;
            const bool rightEchoInvalid = !isfinite(rightCm);
            const bool rightInvalidDuringShift = shifting && rightEchoInvalid && !(leftCm >= 2.6f && leftCm <= 8.0f);
            if (stale) {
                char cause[128];
                snprintf(cause, sizeof(cause), "sensores desatualizados: idade=%lums limite=%lums",
                         static_cast<unsigned long>(sensorNowMs - snapshot.sampledAtMs),
                         static_cast<unsigned long>(MAX_LATERAL_SAMPLE_AGE_MS));
                safeStop(cause);
                motionResultPending = false;
                return;
            }
            if (guidedForward && guidedRecovery.phase != RecoveryPhase::Braking) {
                const auto lateral = guidedLateralClearance.update(snapshot);
                const bool recovering = adjusting || guidedRecovery.phase == RecoveryPhase::Resuming;
                float offsetWithReference = guidedRecovery.steeringOffset(leftCm, rightCm);
                const bool leftNeedsReference = lateral.leftUncertain || (recovering && guidedRecovery.leftBlocked);
                const bool rightNeedsReference = lateral.rightUncertain || (recovering && guidedRecovery.rightBlocked);
                const bool hasOppositeWall = guidedLateralClearance.oppositeWallOffset(
                    leftNeedsReference, rightNeedsReference, leftCm, rightCm, offsetWithReference);
                // Uma saída curta pode começar junto à parede que motivou o ajuste.
                // Só libere os primeiros milímetros se o outro lado for uma referência válida.
                const float oppositeCm = guidedRecovery.leftBlocked ? rightCm : leftCm;
                const bool leavingContact = adjusting && (guidedRecovery.leftBlocked != guidedRecovery.rightBlocked) &&
                    hasOppositeWall && oppositeCm >= 2.6f &&
                    motion.getDistanceTravelledMm() < 6.0f;
                const bool leftBlocked = lateral.leftBlocked &&
                    !(leavingContact && guidedRecovery.leftBlocked && !lateral.rightBlocked);
                const bool rightBlocked = lateral.rightBlocked &&
                    !(leavingContact && guidedRecovery.rightBlocked && !lateral.leftBlocked);
                if (frontBlocked || leftBlocked || rightBlocked) {
                    if (!adjusting)
                        guidedRecovery.request(frontBlocked, leftBlocked, rightBlocked, guidedFrontApproach);
                    motion.brakeForObstacle();
                    if (!adjusting)
                        Serial.printf("RECUPERACAO GUIADA: freando; frente=%.2fcm esquerda=%.2fcm direita=%.2fcm\n",
                                      snapshot.front.distanceCm, snapshot.left.distanceCm, snapshot.right.distanceCm);
                } else {
                    float offset = hasOppositeWall ? offsetWithReference : lateral.yawOffsetDeg;
                    if (leavingContact)
                        offset = guidedRecovery.leftBlocked ? fminf(-2.0f, offset) : fmaxf(2.0f, offset);
                    // As distâncias laterais estão oblíquas durante o avanço diagonal.
                    if (shifting && !recovering && !hasOppositeWall) offset = 0.0f;
                    motion.setGuidedLateralSteering(lateral.slow || recovering || shifting || hasOppositeWall,
                                                   cornerSequenceActive() ? offset * cornerTurnDirection : offset);
                    if (shifting && !adjusting && !rightEchoInvalid && !motion.isSensorPaused() &&
                        nav::guidedCornerBrakeAtRightTarget(snapshot.right.distanceCm, lateral.rightProjectedCm))
                        motion.brakeAtFrontTarget();
                    if (guidedPivotRecovery.phase == GuidedPivotRecovery::Phase::Adjusting &&
                        !rightEchoInvalid && nav::guidedCornerRightGapCm(rightCm) <= nav::GUIDED_CORNER_RIGHT_MIN_CM &&
                        !motion.isSensorPaused())
                        motion.brakeAtFrontTarget();
                }
            }
            if (!guidedForward) {
                using PivotPhase = GuidedPivotRecovery::Phase;
                const bool returning = guidedPivotRecovery.phase == PivotPhase::Returning;
                const auto type = motion.getMotionType();
                const bool turning = type == MotionType::TurnLeft90 || type == MotionType::TurnRight90 ||
                    type == MotionType::TurnAround180;
                if (turning || retreating) {
                    const auto lateral = guidedLateralClearance.update(snapshot);
                    const bool frontTooClose = snapshot.front.tooClose && turning &&
                        !(returning && guidedPivotRecovery.frontBlocked);
                    const bool leftTooClose = lateral.leftBlocked &&
                        !(returning ? guidedPivotRecovery.leftBlocked : retreating && guidedRecovery.leftBlocked);
                    const bool rightTooClose = lateral.rightBlocked &&
                        !(returning ? guidedPivotRecovery.rightBlocked : retreating && guidedRecovery.rightBlocked);
                    if (frontTooClose || leftTooClose || rightTooClose) {
                        if (turning && cornerSequenceActive() && guidedPivotRecovery.phase == PivotPhase::None &&
                            guidedRoute.action() == nav::GuidedAction::FinishLeftTurn) {
                            guidedPivotRecovery = {PivotPhase::BeforeReturn, frontTooClose, leftTooClose, rightTooClose};
                            Serial.printf("CURVA: proximidade no giro; frente=%u esquerda=%u direita=%u; freando para ajustar\n",
                                          frontTooClose, leftTooClose, rightTooClose);
                        }
                        motion.brakeForObstacle();
                    }
                }
            }
            if ((motion.isMovingForward() || retreating) && guidedRecovery.phase != RecoveryPhase::Braking) {
                const auto decision=frontEchoRecovery.update(!frontInvalid && !rightInvalidDuringShift && !retreatFrontInvalid,
                    snapshot.sampledAtMs,millis(),
                    motion.sensorPauseSettled());
                if(decision==FrontEchoRecovery::Decision::Failed) {
                    mission.fail();
                    safeStop(retreating ? "recuperacao guiada: eco frontal invalido ou encurtando durante a re"
                             : shifting ? "curva: eco frontal ou direito instavel durante a diagonal"
                                      : "eco frontal instavel: recuperacao excedeu 1200ms ou 3 pausas no trecho");
                    motionResultPending=false;return;
                }
                const bool hold=decision==FrontEchoRecovery::Decision::Hold;
                const bool wasPaused=motion.isSensorPaused();
                if(!motion.setSensorPaused(hold)) {
                    mission.fail();safeStop("falha ao pausar/retomar apos perda do eco frontal");
                    motionResultPending=false;return;
                }
                if(hold!=wasPaused) {
                    if(hold && frontEchoRecoveries<65535)++frontEchoRecoveries;
                    Serial.println(retreating
                        ? (hold ? "RECUPERACAO GUIADA: re pausada para conferir o eco frontal"
                                : "RECUPERACAO GUIADA: eco frontal confirmado; retomando a re")
                        : shifting
                        ? (hold ? "CURVA: freando e aguardando os ecos frontal e direito"
                                : "CURVA: ecos frontal e direito validos; retomando a diagonal")
                        : (hold ? "ECO FRONTAL: freando e aguardando novas leituras"
                                : "ECO FRONTAL: duas leituras validas; retomando o restante do trecho"));
                    publishRobotStatus(millis(),true);
                }
            }
            if(automaticSlow && !motion.isSensorPaused() && filteredFront.valid &&
               filteredFront.distanceCm>FrontApproach::TARGET_CM+FrontApproach::TOLERANCE_CM &&
               filteredFront.distanceCm<=5.0f &&
               FrontApproach::positionMatchesCell(missionLinearProgressMm + motion.getDistanceTravelledMm(),mazeConfig.cellSizeMm)) {
                // A brake in progress may finish; geometry is checked again afterwards.
                if(motion.refineFrontApproachTarget(filteredFront.distanceCm, missionLinearProgressMm))automaticFrontReferenceUsed=true;
            }
            if(automaticForward && motion.isFrontApproachMode() && !motion.isSensorPaused() &&
               FrontApproach::atTarget(filteredFront.valid,filteredFront.distanceCm)) {
                const float advanceMm = missionLinearProgressMm + motion.getDistanceTravelledMm();
                if(!FrontApproach::positionMatchesCell(advanceMm,mazeConfig.cellSizeMm)) {
                    char cause[128];
                    snprintf(cause,sizeof(cause),"parede antes do centro: eixo avancou %.1fmm; celula=%umm; frente=%.2fcm",
                        advanceMm,mazeConfig.cellSizeMm,filteredFront.distanceCm);
                    mission.fail();safeStop(cause);motionResultPending=false;return;
                }
                automaticFrontReferenceUsed=true;
                motion.brakeAtFrontTarget();
            }
            if (!motion.isSensorPaused() && guidedRecovery.phase != RecoveryPhase::Braking &&
                guidedFrontApproach && filteredFront.valid &&
                filteredFront.distanceCm <= MazeSettings::GUIDED_FRONT_TARGET_CM + MazeSettings::GUIDED_FRONT_TOLERANCE_CM)
                motion.brakeAtFrontTarget();
            const uint32_t nowUs = micros();
            const uint32_t elapsedUs = static_cast<uint32_t>(nowUs - lastControlUpdateUs);
            if (elapsedUs >= CONTROL_PERIOD_US) {
                motion.update(static_cast<float>(elapsedUs) / 1000000.0f);
                lastControlUpdateUs = nowUs;
            }
            if (motion.getStatus() == MotionStatus::Running) {
                delayMicroseconds(500); // evita polling contínuo sem atrasar o ciclo PID
                return;
            }
            status = motion.getStatus();
        }

        motionResultPending = false;
        Serial.printf("MOVIMENTO status=%u motivo=%u distancia=%.2fmm yaw=%.2fgraus\n",
            static_cast<unsigned>(status), static_cast<unsigned>(motion.getFailureReason()),
            motion.getDistanceTravelledMm(), motion.getTurnAngleDeg());
        const bool linear = motion.getMotionType() == MotionType::ForwardCell;
        const bool sequence = cornerSequenceActive();
        const bool recoverableStop = status == MotionStatus::Failed && linear && (sequence || missionMovingForward) &&
            guidedLinearInterruption(motion.getFailureReason());
        if (recoverableStop && guidedRecovery.phase != RecoveryPhase::Braking &&
            guidedRecovery.phase != RecoveryPhase::Adjusting) {
            const auto lateral = guidedLateralClearance.update(motionSnapshot());
            guidedRecovery.request(false, lateral.leftBlocked, lateral.rightBlocked, guidedFrontApproach);
            Serial.println("RECUPERACAO GUIADA: avanço sem progresso no sentido comandado");
        }
        if (linear && (status == MotionStatus::Succeeded || recoverableStop)) {
            const float travelledMm = motion.getDistanceTravelledMm();
            if (sequence) {
                if (guidedRoute.step() <= 2) guidedCornerRemainingMm -= travelledMm;
                if (guidedRoute.action() == nav::GuidedAction::ShiftIntoCorner)
                    guidedCornerShiftTravelMm += travelledMm;
            } else {
                missionLinearProgressMm += travelledMm;
            }
        }
        if (handleGuidedRecoveryResult(status)) return;
        guidedRecovery = {};
        guidedFrontApproach = false;
        motion.setFrontApproachMode(false);
        if (sequence) {
            // A etapa de esquina só termina com a referência frontal confirmada em repouso.
            if (status != MotionStatus::Succeeded || guidedRoute.action() != nav::GuidedAction::ApproachFrontWall)
                guidedRoute.finishMovement(status == MotionStatus::Succeeded);
            guidedPauseStartedMs = millis();
            if (guidedRoute.action() == nav::GuidedAction::Failed) {
                const auto reason = motion.getFailureReason();
                if (reason == MotionFailureReason::LeftWheelStalled)
                    safeStop("teste guiado: roda esquerda sem contagens novas no encoder");
                else if (reason == MotionFailureReason::RightWheelStalled)
                    safeStop("teste guiado: roda direita sem contagens novas no encoder");
                else if (reason == MotionFailureReason::Obstacle)
                    safeStop("teste guiado: parede dentro da area de giro do chassi");
                else if (reason == MotionFailureReason::MotionTimeout && motion.getMotionType() == MotionType::TurnLeft90)
                    safeStop("teste guiado: giro sem progresso angular ou tempo limite atingido");
                else
                    safeStop("falha no teste guiado");
            } else if (guidedRoute.action() == nav::GuidedAction::Complete) {
                ready = false;
                guidedRouteActive = false;
                motors.setEnabled(false);
                Serial.println("TESTE GUIADO CONCLUIDO; motores OFF. Reinicie/reposicione antes de outra tentativa.");
            }
            return;
        }
        finishMissionMotion(status == MotionStatus::Succeeded);
        return;
    }

    if (cornerSequenceActive() || guidedRecovery.phase != GuidedWallRecovery::Phase::None) {
        if (millis() - guidedPauseStartedMs < 1000) { delay(1); return; }
        const auto snapshot = motionSnapshot();
        const uint32_t sensorNowMs = millis();
        if (!snapshot.ready || sensorNowMs - snapshot.sampledAtMs > MAX_LATERAL_SAMPLE_AGE_MS) {
            safeStop("sensores desatualizados antes da etapa guiada"); return;
        }
        const auto action = cornerSequenceActive() ? guidedRoute.action() : nav::GuidedAction::ForwardCell;
        const auto front = snapshot.frontEstimate(sensorNowMs);
        const bool frontValid = snapshot.front.valid && !snapshot.front.tooClose && snapshot.front.pulseUs &&
            isfinite(snapshot.front.distanceCm) && front.valid && isfinite(front.distanceCm);
        if (missionCornerActive && action == nav::GuidedAction::ForwardCell) {
            const float frontCm = fminf(front.distanceCm, snapshot.front.distanceCm);
            if (!frontValid || !nav::guidedLeftOpening(frontCm, mazeConfig.cellSizeMm)) {
                safeStop("missao: abertura frontal nao confirmada apos a curva");
                return;
            }
            missionExitCreditMm = nav::guidedCornerExitProgressMm(guidedCornerShiftTravelMm);
            missionCornerActive = false;
            guidedLateralClearance.reset();
            Serial.printf("MISSAO: curva concluida; avancando %.1fmm apos a diagonal\n",
                          mazeConfig.cellSizeMm - missionExitCreditMm);
            finishMissionMotion(true, true);
            return;
        }
        using PivotPhase = GuidedPivotRecovery::Phase;
        if (guidedPivotRecovery.phase == PivotPhase::BeforeReturn) {
            motors.setEnabled(true);
            if (!motion.startGuidedPivotReturn()) {
                safeStop("curva: falta percurso angular confirmado para desfazer o giro");
                return;
            }
            guidedLateralClearance.reset();
            guidedPivotRecovery.phase = PivotPhase::Returning;
            trackStartedMotion();
            Serial.println("CURVA: desfazendo o giro parcial pelo arco percorrido");
            return;
        }
        if (guidedPivotRecovery.phase == PivotPhase::BeforeAdjustment) {
            const auto left = snapshot.leftEstimate(sensorNowMs);
            const auto right = snapshot.rightEstimate(sensorNowMs);
            const float leftCm = snapshot.left.valid && !snapshot.left.tooClose && left.valid ? left.distanceCm : NAN;
            const float rightCm = snapshot.right.valid && !snapshot.right.tooClose && right.valid
                ? fminf(right.distanceCm, snapshot.right.distanceCm) : NAN;
            const bool rightBlocked = guidedPivotRecovery.rightBlocked;
            const bool leftBlocked = guidedPivotRecovery.leftBlocked ||
                (guidedPivotRecovery.frontBlocked && !rightBlocked);
            float offsetDeg = 0.0f;
            const bool hasOppositeWall = guidedLateralClearance.oppositeWallOffset(
                leftBlocked, rightBlocked, leftCm, rightCm, offsetDeg);
            const float adjustMm = frontValid && hasOppositeWall
                ? nav::guidedCornerRepositionMm(rightBlocked, fminf(front.distanceCm, snapshot.front.distanceCm),
                                              rightCm, guidedCornerShiftTravelMm)
                : 0.0f;
            if (adjustMm == 0.0f) {
                char cause[128];
                snprintf(cause, sizeof(cause), "curva: sem espaco confirmado para ajustar; esquerda=%.2fcm direita=%.2fcm",
                         leftCm, rightCm);
                safeStop(cause);
                return;
            }
            guidedRecovery.request(false, leftBlocked, rightBlocked, false);
            guidedRecovery.planAdjustment(adjustMm);
            guidedPivotRecovery.phase = PivotPhase::Adjusting;
            Serial.printf("CURVA: ajuste pela parede %s; percurso %+.1fmm; correcao estimada %.1fgraus\n",
                          leftBlocked ? "direita" : "esquerda", adjustMm, offsetDeg);
            return;
        }
        if (guidedRecovery.phase == GuidedWallRecovery::Phase::BeforeAdjustment) {
            const float signedMm = guidedRecovery.plannedAdjustmentMm;
            const bool reversing = signedMm < 0.0f;
            const bool frontInBlindZone = reversing && guidedRecovery.frontBlocked &&
                snapshot.front.tooClose && snapshot.front.pulseUs;
            const float frontBeforeCm = frontValid ? fminf(front.distanceCm, snapshot.front.distanceCm) : NAN;
            if ((!frontValid && !frontInBlindZone) || (!reversing && frontBeforeCm * 10.0f < 52.0f + signedMm)) {
                safeStop("recuperacao guiada: eco frontal invalido ou sem espaco para o ajuste");
                return;
            }
            guidedFrontApproach = false;
            motion.setFrontApproachMode(!reversing);
            motors.setEnabled(true);
            // Com eco abaixo da faixa, uma ré curta e já percorrida libera a face do sensor.
            const float adjustmentMm = frontInBlindZone ? fminf(-signedMm, 10.0f) : fabsf(signedMm);
            const bool started = reversing ? motion.startGuidedRetreat(adjustmentMm) : motion.startMoveLinear(adjustmentMm);
            if (!started) { safeStop("recuperacao guiada: falha ao iniciar ajuste"); return; }
            guidedRecovery.startAdjustment(frontBeforeCm);
            trackStartedMotion();
            Serial.printf("RECUPERACAO GUIADA: %s %.1fmm; frente=%.2fcm\n",
                          reversing ? "recuo pelo trecho percorrido" : "avanco para sair do travamento",
                          adjustmentMm, frontBeforeCm);
            return;
        }
        if (guidedRecovery.phase == GuidedWallRecovery::Phase::Checking) {
            const float frontMinCm = guidedRecovery.approach ? MazeSettings::GUIDED_FRONT_EMERGENCY_CM : 5.0f;
            const auto lateral = guidedLateralClearance.update(snapshot);
            const auto left = snapshot.leftEstimate(sensorNowMs);
            const auto right = snapshot.rightEstimate(sensorNowMs);
            const float leftCm = snapshot.left.valid && left.valid ? left.distanceCm : NAN;
            const float rightCm = snapshot.right.valid && right.valid ? right.distanceCm : NAN;
            const float frontCm = fminf(front.distanceCm, snapshot.front.distanceCm);
            if (!frontValid && guidedRecovery.frontBlocked && snapshot.front.tooClose &&
                snapshot.front.pulseUs && guidedRecovery.failedAdjustmentMm == 0.0f &&
                !lateral.leftBlocked && !lateral.rightBlocked && guidedRecovery.availableRetreatMm() >= 5.0f) {
                guidedRecovery.planAdjustment(-fminf(10.0f, guidedRecovery.availableRetreatMm()));
                return;
            }
            if (!frontValid || (lateral.leftBlocked && lateral.rightBlocked)) {
                safeStop("recuperacao guiada: falta referencia frontal ou espaco nas laterais");
                return;
            }
            float yawOffset = guidedRecovery.steeringOffset(leftCm, rightCm);
            const bool hasOppositeWall = guidedLateralClearance.oppositeWallOffset(
                guidedRecovery.leftBlocked, guidedRecovery.rightBlocked, leftCm, rightCm, yawOffset);
            const bool knownSideStillClose =
                (lateral.leftBlocked && guidedRecovery.leftBlocked && !lateral.rightBlocked) ||
                (lateral.rightBlocked && guidedRecovery.rightBlocked && !lateral.leftBlocked);
            const float oppositeCm = guidedRecovery.leftBlocked ? rightCm : leftCm;
            const float forwardSpaceMm = (frontCm - 5.0f) * 10.0f - 2.0f;
            if (guidedRecovery.failedAdjustmentMm < 0.0f) {
                // O último movimento para trás travou. Avance devagar dentro do trecho pendente.
                const float advanceMm = fminf(15.0f, fminf(forwardSpaceMm, guidedRecovery.remainingMm));
                if (advanceMm < 5.0f || ((lateral.leftBlocked || lateral.rightBlocked) &&
                                         (!knownSideStillClose || !hasOppositeWall || oppositeCm < 2.6f))) {
                    safeStop("recuperacao guiada: sem espaco confirmado para avancar apos travar na re");
                    return;
                }
                guidedRecovery.planAdjustment(advanceMm);
                return;
            }
            if (guidedRecovery.failedAdjustmentMm > 0.0f || knownSideStillClose || frontCm <= frontMinCm) {
                const float retreatMm = fminf(15.0f, guidedRecovery.availableRetreatMm());
                if (retreatMm < 5.0f) {
                    safeStop("recuperacao guiada: sem trecho percorrido para recuar mais");
                    return;
                }
                guidedRecovery.planAdjustment(-retreatMm);
                return;
            }
            if (lateral.leftBlocked || lateral.rightBlocked ||
                (!hasOppositeWall && ((guidedRecovery.leftBlocked && !isfinite(leftCm)) ||
                                     (guidedRecovery.rightBlocked && !isfinite(rightCm))))) {
                safeStop("recuperacao guiada: parede lateral de referencia indisponivel");
                return;
            }
            if (action == nav::GuidedAction::ShiftIntoCorner &&
                nav::guidedCornerShiftReached(guidedCornerShiftTravelMm) && snapshot.right.valid && right.valid &&
                nav::guidedCornerRightGapCm(fminf(right.distanceCm, snapshot.right.distanceCm)) >=
                    nav::GUIDED_CORNER_RIGHT_MIN_CM &&
                nav::guidedCornerBrakeAtRightTarget(snapshot.right.distanceCm, NAN)) {
                guidedRecovery = {};
                guidedRoute.finishMovement(true);
                Serial.printf("CURVA: diagonal preservada apos a re (%.1fmm); verificando giro final\n",
                              guidedCornerShiftTravelMm);
                return;
            }
            const bool startsAtCellCentre = action == nav::GuidedAction::ForwardCell &&
                (!cornerSequenceActive() || guidedRoute.step() < 2) &&
                guidedRecovery.originalTargetMm() >= mazeConfig.cellSizeMm - 5.0f;
            const float centreRetreatMm = guidedRecovery.centreRetreatMm(startsAtCellCentre, forwardSpaceMm);
            if (centreRetreatMm >= 4.0f) {
                guidedRecovery.planAdjustment(-centreRetreatMm);
                Serial.printf("RECUPERACAO GUIADA: buscando o centro anterior; re %.1fmm\n", centreRetreatMm);
                return;
            }
            guidedFrontApproach = guidedRecovery.approach;
            motion.setFrontApproachMode(true);
            const float remainingMm = guidedRecovery.remainingMm;
            if (remainingMm <= 1.0f && remainingMm >= -2.0f) {
                guidedFrontApproach = false;
                motion.setFrontApproachMode(false);
                guidedRecovery = {};
                if (!cornerSequenceActive()) finishMissionMotion(true);
                else if (action != nav::GuidedAction::ApproachFrontWall) guidedRoute.finishMovement(true);
                return;
            }
            motors.setEnabled(true);
            if (remainingMm < 0.0f || !motion.startMoveLinear(remainingMm)) {
                safeStop("recuperacao guiada: distancia pendente invalida");
                return;
            }
            guidedRecovery.phase = GuidedWallRecovery::Phase::Resuming;
            trackStartedMotion();
            Serial.printf("RECUPERACAO GUIADA: retomando %.1fmm, ajuste de rumo %.1fgraus\n",
                          remainingMm, yawOffset);
            return;
        }
        if (action == nav::GuidedAction::ApproachFrontWall) {
            if (!frontValid || snapshot.front.distanceCm <= MazeSettings::GUIDED_FRONT_EMERGENCY_CM) {
                safeStop("aproximacao frontal invalida ou sem reserva"); return;
            }
            const auto plan = nav::planGuidedCorner(front.distanceCm, mazeConfig.cellSizeMm, guidedCornerRemainingMm);
            if (plan.action == nav::GuidedCornerAction::AtTarget) {
                if (!nav::guidedFrontAtCorner(snapshot.front.distanceCm)) {
                    safeStop("eco frontal nao confirmou a posicao de curva"); return;
                }
                guidedRoute.finishMovement(true); guidedPauseStartedMs = millis();
                Serial.printf("APROXIMACAO CONFIRMADA: frente=%.2fcm\n", front.distanceCm);
                return;
            }
            if (plan.action == nav::GuidedCornerAction::TooClose ||
                snapshot.front.distanceCm < MazeSettings::GUIDED_FRONT_TARGET_CM - MazeSettings::GUIDED_FRONT_TOLERANCE_CM) {
                safeStop("esquina: frente abaixo da faixa de3cm para o giro"); return;
            }
            if (plan.action == nav::GuidedCornerAction::NoTravelLeft) {
                char cause[128];
                snprintf(cause, sizeof(cause), "esquina nao confirmada: frente=%.2fcm; limite de avanco atingido",
                         front.distanceCm);
                safeStop(cause); return;
            }
            guidedFrontApproach = plan.action == nav::GuidedCornerAction::ApproachSlowly;
            if (!guidedFrontApproach && snapshot.front.distanceCm <= 5.0f) {
                safeStop("esquina: eco frontal curto antes do avanco"); return;
            }
            motion.setFrontApproachMode(guidedFrontApproach);
            motors.setEnabled(true);
            if (!motion.startMoveLinear(plan.distanceMm)) { safeStop("falha ao iniciar aproximacao"); return; }
            trackStartedMotion();
            Serial.printf("%s: %s; frente=%.2fcm; trecho=%.1fmm; restante=%.1fmm\n",
                          missionCornerActive ? "MISSAO CURVA" : "TESTE ETAPA 3/5",
                          guidedFrontApproach ? "aproximacao lenta" : "avanco ate a esquina",
                          front.distanceCm, plan.distanceMm, guidedCornerRemainingMm);
            return;
        }
        const bool forward = action == nav::GuidedAction::ForwardCell;
        if (forward && guidedRoute.step() < 2) {
            if (!frontValid) {
                safeStop("frente invalida antes do avanco guiado"); return;
            }
            if (front.distanceCm <= FrontApproach::START_CM) {
                guidedRoute.resetAtCorner(); guidedPauseStartedMs = millis();
                Serial.println("ESQUINA: transicao antecipada para aproximacao lenta; sem insistir em celula inteira.");
                return;
            }
            const float segmentMm = nav::guidedPreCornerMoveMm(front.distanceCm, mazeConfig.cellSizeMm);
            if (snapshot.front.distanceCm <= 5.0f) {
                safeStop("eco frontal curto antes do avanco guiado"); return;
            }
            motion.setFrontApproachMode(false);
            motors.setEnabled(true);
            if (!motion.startMoveLinear(segmentMm)) { safeStop("falha ao iniciar trecho antes da curva"); return; }
            trackStartedMotion();
            Serial.printf("TESTE ETAPA %u/5: avancar ate%.1fmm, preservando8cm antes da aproximacao\n",
                          guidedRoute.step() + 1, segmentMm);
            return;
        }
        if (forward && (!frontValid || snapshot.front.distanceCm <= 5.0f || front.distanceCm <= 5.0f)) {
            safeStop("frente invalida ou obstaculo antes do avanco apos o giro"); return;
        }
        const bool turnEntry = action == nav::GuidedAction::BeginLeftTurn;
        if (turnEntry) {
            if (!frontValid || !nav::guidedFrontAtCorner(front.distanceCm) ||
                !nav::guidedFrontAtCorner(snapshot.front.distanceCm)) {
                safeStop("posicao frontal da esquina nao confirmada antes do giro"); return;
            }
            const auto left = snapshot.leftEstimate(sensorNowMs);
            if (!snapshot.left.valid || !left.valid ||
                !nav::guidedLeftOpening(snapshot.left.distanceCm, mazeConfig.cellSizeMm) ||
                !nav::guidedLeftOpening(left.distanceCm, mazeConfig.cellSizeMm)) {
                safeStop("abertura de saida nao confirmada antes do giro"); return;
            }
        }
        const auto right = snapshot.rightEstimate(sensorNowMs);
        const float rightCm = fminf(right.distanceCm, snapshot.right.distanceCm);
        const char* turnLogPrefix = missionCornerActive ? "MISSAO CURVA" : "TESTE ETAPA 4/5";
        const bool rightValid = snapshot.right.valid && !snapshot.right.tooClose && right.valid &&
            isfinite(snapshot.right.distanceCm) && isfinite(right.distanceCm);
        if (action == nav::GuidedAction::ShiftIntoCorner) {
            const float frontCm = fminf(front.distanceCm, snapshot.front.distanceCm);
            const float shiftMm = frontValid && rightValid && !snapshot.left.tooClose
                ? nav::guidedCornerShiftMm(frontCm, rightCm) : 0.0f;
            if (shiftMm == 0.0f) {
                char cause[128];
                snprintf(cause, sizeof(cause), "curva: avanco diagonal sem folga; frente=%.2fcm direita=%.2fcm",
                         frontCm, rightCm);
                safeStop(cause);
                return;
            }
            motion.setFrontApproachMode(true);
            motors.setEnabled(true);
            if (!motion.startMoveLinear(shiftMm)) { safeStop("falha ao iniciar avanco diagonal"); return; }
            trackStartedMotion();
            Serial.printf("%s: avanco diagonal ate %.1fmm; frente=%.2fcm lado externo=%.2fcm; alvo perpendicular=%.2fcm\n",
                          turnLogPrefix, shiftMm, frontCm, rightCm, nav::GUIDED_CORNER_RIGHT_TARGET_CM);
            return;
        }
        if (turnEntry || action == nav::GuidedAction::FinishLeftTurn) {
            const bool leftTooClose = snapshot.left.tooClose;
            const bool rightTooClose = snapshot.right.tooClose ||
                (rightValid && nav::guidedCornerRightGapCm(rightCm) < nav::GUIDED_CORNER_RIGHT_MIN_CM);
            if (!turnEntry && nav::guidedCornerShiftReached(guidedCornerShiftTravelMm) &&
                leftTooClose != rightTooClose) {
                guidedPivotRecovery = {PivotPhase::BeforeAdjustment, false, leftTooClose, rightTooClose};
                Serial.println("CURVA: ajustando a folga lateral antes do giro final");
                return;
            }
            if (!turnEntry && (!nav::guidedCornerShiftReached(guidedCornerShiftTravelMm) ||
                               !frontValid || snapshot.front.distanceCm <= 5.0f || front.distanceCm <= 5.0f ||
                               !rightValid || rightTooClose || leftTooClose)) {
                char cause[128];
                snprintf(cause, sizeof(cause), "curva: giro final sem folga; diagonal=%.1fmm frente=%.2fcm direita=%.2fcm",
                         guidedCornerShiftTravelMm, front.distanceCm, rightCm);
                safeStop(cause);
                return;
            }
            guidedLateralClearance.reset();
            const float degrees = turnEntry ? nav::GUIDED_CORNER_ENTRY_DEG : nav::GUIDED_CORNER_EXIT_DEG;
            motors.setEnabled(true);
            if (!motion.startGuidedPivot(degrees * cornerTurnDirection)) { safeStop("falha ao iniciar giro parcial"); return; }
            trackStartedMotion();
            if (turnEntry)
                Serial.printf("%s: primeiro giro a %s %.0fgraus\n", turnLogPrefix,
                              cornerTurnDirection > 0 ? "esquerda" : "direita", degrees);
            else
                Serial.printf("%s: segundo giro a %s %.0fgraus; diagonal=%.1fmm interior=%.2fcm exterior=%.2fcm; folga estimada=%.2fcm\n",
                              turnLogPrefix, cornerTurnDirection > 0 ? "esquerda" : "direita", degrees,
                              guidedCornerShiftTravelMm, snapshot.left.distanceCm, rightCm,
                              nav::guidedCornerRightGapCm(rightCm));
            return;
        }
        const float exitMm = mazeConfig.cellSizeMm - nav::guidedCornerExitProgressMm(guidedCornerShiftTravelMm);
        motors.setEnabled(true);
        if (!motion.startMoveLinear(exitMm)) { safeStop("falha ao iniciar avanco apos o giro"); return; }
        trackStartedMotion();
        Serial.printf("TESTE ETAPA 5/5: avancar %.1fmm apos a curva\n", exitMm);
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
