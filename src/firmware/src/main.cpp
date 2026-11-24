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
#include "control/straight_proximity_guard.hpp"
#include "control/corner_sonar_gate.hpp"
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
StraightProximityGuard straightProximity;
uint16_t frontEchoRecoveries=0;
bool ready = false;
bool hardwareReady = false;
bool attemptUsed = false;
bool guidedRouteActive = false;
bool missionCornerActive = false;
enum class RollingPhase { None, BeforeRetreat, Retreat, BeforeArc, Arc, ConfirmExit, WaitSonarPause, RecoverRetreat };
// Curva da missão em arco aberto (mesmos parâmetros do teste 'a'/'A').
uint8_t rollingRecoveries=0;
// Recuperação da reta da missão (ré curta e retomada).
uint8_t missionStraightRetries=0;
bool missionRetreatPending=false;
// Posição do eixo em relação ao centro da célula ao chegar (negativo = ficou antes,
// ex.: parede na frente impede chegar ao centro). A ré seguinte desconta isso para
// voltar ao centro da célula anterior em vez de passar do ponto.
float missionArrivalOffsetMm=0.0f;
// Os sensores laterais ficam na frente do robô, ~85 mm à frente do eixo (labirinto, 23/11):
// com o eixo no centro da célula eles olham a divisa com a próxima. As paredes laterais
// de cada célula são medidas na reta, quando esses sensores passam pelo centro dela.
constexpr float SIDE_SENSOR_AHEAD_MM=85.0f;
constexpr uint8_t SIDE_SAMPLES_MAX=24;
uint16_t sideSamplesLeft[SIDE_SAMPLES_MAX], sideSamplesRight[SIDE_SAMPLES_MAX];
uint8_t sideSampleCount=0, sideValidLeft=0, sideValidRight=0;
uint32_t lastSideSampleStamp=0;
float missionBackCompMm=0.0f;
// Antes da ré: alinhar o rumo com o corredor e só então recuar.
bool missionAlignPending=false;
// Recuperação geral: falhas durante a missão não encerram a tentativa.
uint8_t missionResumeCount=0;
// Depois de um arco o lado de dentro termina junto da ponta da parede interna (invisível
// aos sonares, que ficam na frente): a reta seguinte começa inclinada para longe dela.
float postAvoidYawDeg=0.0f;
constexpr float POST_AVOID_DEG=8.0f;
constexpr float POST_AVOID_MM=40.0f;
constexpr uint8_t MISSION_RESUME_MAX=10;
bool resumeInProgress=false;
float missionPendingBackMm=0.0f;
float rollingDoneDeg=0.0f;
Pose2D rollingArcStartPose{};
RollingPhase rollingPhase=RollingPhase::None;
constexpr float ROLLING_RETREAT_MM=30.0f;
float rollingRadiusMm=30.0f;
float rollingPlannedRetreatMm=30.0f;
CornerSonarGate rollingSonarGate;
CellObservationGate rollingEntryGate;
float rollingExitProgressMm=0.0f;
bool rollingPathWarningSent=false;
int rollingDirection=1;
uint32_t rollingPhaseMs=0, rollingPauseMs=0, rollingExitStamp=0;
unsigned rollingExitVotes=0;
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
// Teste de curva em arco aberto (comandos 'a'/'A'). Geometria simulada:
// com eixo a 86 mm da frente e 125 mm de largura, arco de 100 mm iniciado
// com o eixo 110 mm antes do centro da célula passa com ~11 mm de folga.
namespace ArcTest {
constexpr float RADIUS_MM = 100.0f;
// Leitura do sonar frontal no início do arco: (85 + 110) mm - eixo→sensor (~85 mm).
constexpr float FRONT_TRIGGER_CM = 11.0f;
// A 11 cm o sensor lateral fica na quina da abertura e a leitura oscila (labirinto, 23/11):
// a abertura é conferida antes, com a frente a 8 cm, e só então o robô recua até 11 cm.
constexpr float SIDE_CHECK_FRONT_CM = 8.0f;
constexpr float FRONT_TOLERANCE_CM = 0.6f; // ajuste fino do motor passa 4–10 mm do alvo
// Lado da curva precisa estar livre. O sensor lateral fica ~40 mm à frente do eixo,
// perto da ponta da parede; o feixe largo pode pegar a quina, então o limite é baixo.
constexpr float SIDE_OPEN_MIN_CM = 8.0f;
constexpr uint8_t MAX_CORRECTIONS = 3;
constexpr uint32_t SETTLE_MS = 400;
constexpr uint32_t LOG_PERIOD_MS = 50;
// Contato no arco (roda travada): recua e completa o ângulo que falta.
constexpr uint8_t MAX_RECOVERIES = 5;
constexpr float RECOVERY_RETREAT_MM = 15.0f;
}
struct ArcDiagnostic {
    enum class Phase : uint8_t { None, Delay, Measure, Approach, Settle, Arc, Recover, Exit };
    Phase phase = Phase::None;
    int dir = 0;              // +1 esquerda, -1 direita
    uint8_t corrections = 0;
    uint8_t recoveries = 0;   // recuos depois de contato durante o arco
    bool sideConfirmed = false; // abertura conferida perto da parede (frente ~8 cm)
    float doneDeg = 0.0f;     // ângulo já girado em trechos anteriores do arco
    uint32_t startedMs = 0;
    uint32_t phaseMs = 0;
    uint32_t lastLogMs = 0;
    bool active() const { return phase != Phase::None; }
} arcDiagnostic;
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

void restoreRollingSonar() {
    if(rollingSonarGate.suspended()){
        rollingSonarGate.resume(millis());
        samplingEnabled.store(true);
        Serial.println("CURVA: ultrassons retomados; aguardando leituras posteriores ao giro");
    }
}

bool tryResumeMission(const char* reason, bool clearMap);
void safeStop(const char *reason, mm::EndReason endReason = mm::EndReason::Error) {
    if (endReason == mm::EndReason::Error && tryResumeMission(reason, false)) return;
    restoreRollingSonar();
    snprintf(lastStopReason,sizeof(lastStopReason),"%s",reason);
    terminalTelemetryPublished = false;
    remoteMissionActive = false;
    remoteState = endReason == mm::EndReason::ManualStop ? "stopped" : "error";

    if (ready && !guidedRouteActive) mission.fail();
    ready = false;
    missionCornerActive = false;
    rollingPhase=RollingPhase::None;
    missionMovingForward = false;
    missionExitCreditMm = 0.0f;
    startDelay.cancel();
    pivotDiagnostic = {};
    arcDiagnostic = {};
    missionRetreatPending = false;
    missionStraightRetries = 0;
    missionArrivalOffsetMm = 0.0f;
    missionBackCompMm = 0.0f;
    missionAlignPending = false;
    postAvoidYawDeg = 0.0f;
    hardwareWallSensor.clearSideOverride();
    sideSampleCount = sideValidLeft = sideValidRight = 0;
    guidedRecovery = {};
    guidedPivotRecovery = {};
    frontEchoRecovery.reset();frontProximity.reset();
    motion.stop();
    motors.brake();
    motors.setEnabled(false);
    finishRoute(endReason);
    Serial.printf("Tentativa interrompida: %s\n", reason);
}

// Em vez de encerrar a missão: freia, recua 15 mm mantendo o rumo, desarma o que estava
// em andamento e volta a observar a célula confirmada (o mapa é mantido). Só a parada
// manual, o limite de tempo e a perda do Bluetooth encerram de fato.
bool tryResumeMission(const char* reason, bool clearMap) {
    if (!ready || guidedRouteActive || resumeInProgress || pivotDiagnostic.active() || arcDiagnostic.active())
        return false;
    if (strstr(reason, "tempo") || strstr(reason, "Bluetooth") || strstr(reason, "parada")) return false;
    if (missionResumeCount >= MISSION_RESUME_MAX) {
        Serial.printf("RECUPERACAO GERAL esgotada (%u tentativas)\n", static_cast<unsigned>(missionResumeCount));
        return false;
    }
    resumeInProgress = true;
    ++missionResumeCount;
    Serial.printf("RECUPERACAO GERAL %u/%u: %s; recuando 15mm e reobservando a celula (%u,%u)\n",
                  static_cast<unsigned>(missionResumeCount), static_cast<unsigned>(MISSION_RESUME_MAX),
                  reason, mission.x(), mission.y());
    restoreRollingSonar();
    samplingEnabled.store(true);
    missionCornerActive = false;
    rollingPhase = RollingPhase::None;
    missionMovingForward = false;
    missionExitCreditMm = missionLinearProgressMm = 0.0f;
    missionRetreatPending = missionAlignPending = false;
    missionStraightRetries = 0;
    missionArrivalOffsetMm = missionBackCompMm = 0.0f;
    hardwareWallSensor.clearSideOverride();
    sideSampleCount = sideValidLeft = sideValidRight = 0;
    guidedRecovery = {};
    guidedPivotRecovery = {};
    frontEchoRecovery.reset(); frontProximity.reset(); straightProximity.reset();
    motionResultPending = false;
    motion.stop();
    motors.brake();
    delay(300);
    motors.setEnabled(true);
    if (motion.startGuidedRetreat(15.0f)) {
        uint32_t lastUs = micros();
        const uint32_t startedMs = millis();
        while (motion.getStatus() == MotionStatus::Running && millis() - startedMs < 2500) {
            const uint32_t nowUs = micros();
            if (nowUs - lastUs >= CONTROL_PERIOD_US) { motion.update((nowUs - lastUs) / 1000000.0f); lastUs = nowUs; }
            delay(1);
        }
        motion.stop();
    }
    // Curva incompleta deixa o robô torto: volta ao rumo do corredor antes de reobservar.
    motors.setEnabled(true);
    if (motion.startAlignToReference(3.0f, 100.0f)) {
        Serial.println("RECUPERACAO GERAL: girando de volta ao rumo do corredor");
        uint32_t lastUs = micros();
        const uint32_t startedMs = millis();
        while (motion.getStatus() == MotionStatus::Running && millis() - startedMs < 5000) {
            const uint32_t nowUs = micros();
            if (nowUs - lastUs >= CONTROL_PERIOD_US) { motion.update((nowUs - lastUs) / 1000000.0f); lastUs = nowUs; }
            delay(1);
        }
        motion.stop();
    }
    motors.brake();
    motors.setEnabled(false);
    if (clearMap) {
        mazeMap.init(mazeMap.width(), mazeMap.height());
        Serial.println("RECUPERACAO GERAL: mapa limpo para reexplorar");
    }
    (void)mission.resumeAtCurrentCell();
    cellObservation.begin(millis());
    resumeInProgress = false;
    return true;
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

uint16_t medianMm(uint16_t* values, uint8_t count) {
    for (uint8_t a = 0; a < count; ++a)
        for (uint8_t b = a + 1; b < count; ++b)
            if (values[b] < values[a]) { const uint16_t v = values[a]; values[a] = values[b]; values[b] = v; }
    return values[count / 2];
}

void handleMissionUpdate(const nav::MissionUpdate &update) {
    using Action = nav::MissionAction;
    const auto action = update.action;
    if (action != Action::None) hardwareWallSensor.clearSideOverride();
    if (update.turnTowardNextCell && (action==Action::TurnLeft90 || action==Action::TurnRight90)) {
        const auto frame=getLateralSnapshot();const auto front=frame.frontEstimate(millis());
        const auto exit=action==Action::TurnLeft90 ? frame.leftEstimate(millis()) : frame.rightEstimate(millis());
        const auto* cell=mazeMap.get(mission.x(),mission.y());
        const auto exitDir=mm::MazeMap::relToAbs(mission.heading(),
            action==Action::TurnLeft90?mm::Rel::Left:mm::Rel::Right);
        const bool mappedCorner=cell && cell->isKnown(mission.heading()) && cell->hasWall(mission.heading()) &&
            cell->isKnown(exitDir) && !cell->hasWall(exitDir);
        if (mappedCorner || (front.valid && exit.valid &&
            nav::missionCornerEligible(front.distanceCm,exit.distanceCm,mazeConfig.cellSizeMm))) {
            const auto back=mm::MazeMap::relToAbs(mission.heading(),mm::Rel::Back);
            const int dx=back==Direction::East?1:back==Direction::West?-1:0;
            const int dy=back==Direction::South?1:back==Direction::North?-1:0;
            const auto* previous=mazeMap.get(int(mission.x())+dx,int(mission.y())+dy);
            if (!cell || !cell->isKnown(back) || cell->hasWall(back) || !previous || !previous->visited) {
                // Depois de limpar o mapa a célula de trás pode não estar marcada; o recuo é curto.
                Serial.println("CURVA COM RECUO: retaguarda nao confirmada no mapa; seguindo assim mesmo");
            }
            rollingDirection=action==Action::TurnLeft90?1:-1;
            rollingPhase=RollingPhase::BeforeRetreat;rollingPhaseMs=millis();rollingPauseMs=0;
            rollingEntryGate.begin(rollingPhaseMs);
            rollingExitVotes=0;rollingExitStamp=0;motionResultPending=false;
            Serial.printf("CURVA COM RECUO: re %.1fmm; arco 90 graus para %s; sem partida automatica extra\n",
                          ROLLING_RETREAT_MM,rollingDirection>0?"esquerda":"direita");return;
        }
    }
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
        if (action == Action::TurnLeft90 || action == Action::TurnRight90 || action == Action::TurnAround180) {
            guidedLateralClearance.reset();
            missionArrivalOffsetMm = 0.0f; // depois do giro o desvio vira lateral
        }
        motors.setEnabled(true);
    }
    switch (action) {
        case nav::MissionAction::None:
            return;
        case nav::MissionAction::MoveForwardOneCell:
            straightProximity.reset();
            missionStraightRetries = 0;
            missionRetreatPending = false;
            sideSampleCount = sideValidLeft = sideValidRight = 0;
            started = motion.startMoveLinear(mazeConfig.cellSizeMm - missionLinearProgressMm);
            break;
        case nav::MissionAction::MoveBackwardOneCell:
        {
            // Se a chegada ficou antes do centro (parede na frente), a ré volta só o que avançou.
            const float cell = static_cast<float>(mazeConfig.cellSizeMm);
            const float back = fmaxf(cell - 60.0f, fminf(cell, cell + missionArrivalOffsetMm));
            missionBackCompMm = back - cell;
            missionArrivalOffsetMm = 0.0f;
            if (motion.startAlignToReference(1.5f)) {
                missionAlignPending = true;
                missionPendingBackMm = back;
                started = true;
                Serial.println("ALINHANDO com o corredor antes da re");
            } else {
                started = motion.startMoveLinear(-back);
            }
            Serial.printf("RE DE UMA CELULA: %.1fmm (compensa chegada %.1fmm antes do centro)\n",
                          back, -missionBackCompMm);
        }
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
            if (update.state != nav::MissionState::Completed &&
                tryResumeMission("falha de navegacao", update.navigationStatus == nav::NavigationStatus::NoPath))
                return;
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
    rollingPhase=RollingPhase::None;
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

// Mediana de 5 disparos de um sonar, com a amostragem de fundo pausada (motores parados).
bool measureIsolatedCm(UltrasonicUS100& sensor, float& cm) {
    float values[5]{};
    int count = 0;
    for (int i = 0; i < 5; ++i) {
        delay(30);
        const auto reading = sensor.measure();
        if (reading.valid) values[count++] = reading.distanceCm;
    }
    if (count < 3) { cm = NAN; return false; }
    for (int a = 0; a < count; ++a)
        for (int b = a + 1; b < count; ++b)
            if (values[b] < values[a]) { const float v = values[a]; values[a] = values[b]; values[b] = v; }
    cm = values[count / 2];
    return true;
}

bool measureAllIsolated(float& front, float& left, float& right) {
    samplingEnabled.store(false);
    const uint32_t requestedAt = millis();
    while (!samplingPaused.load() && millis() - requestedAt < 200) delay(1);
    if (!samplingPaused.load()) { samplingEnabled.store(true); return false; }
    measureIsolatedCm(sensorFront, front);
    measureIsolatedCm(sensorLeft, left);
    measureIsolatedCm(sensorRight, right);
    samplingEnabled.store(true);
    return true;
}

const char* arcPhaseName(ArcDiagnostic::Phase phase) {
    switch (phase) {
        case ArcDiagnostic::Phase::Delay: return "ESPERA";
        case ArcDiagnostic::Phase::Measure: return "MEDIR";
        case ArcDiagnostic::Phase::Approach: return "APROXIMAR";
        case ArcDiagnostic::Phase::Settle: return "ASSENTAR";
        case ArcDiagnostic::Phase::Arc: return "ARCO";
        case ArcDiagnostic::Phase::Recover: return "RECUO";
        case ArcDiagnostic::Phase::Exit: return "SAIDA";
        default: return "-";
    }
}

// Linha de debug completa: fase, sonares filtrados, encoders, velocidades, IMU e pose.
void logArcDebug(uint32_t nowMs, bool force = false) {
    if (!force && nowMs - arcDiagnostic.lastLogMs < ArcTest::LOG_PERIOD_MS) return;
    arcDiagnostic.lastLogMs = nowMs;
    const auto frame = getLateralSnapshot();
    const auto f = frame.frontEstimate(nowMs), l = frame.leftEstimate(nowMs), r = frame.rightEstimate(nowMs);
    const auto travel = motion.getWheelTravel();
    const auto pose = motion.getPose();
    Serial.printf("DBG t=%lu fase=%s F=%.1f L=%.1f R=%.1f encE=%.1f encD=%.1f "
                  "velE=%.0f/%.0f velD=%.0f/%.0f ang=%.2f x=%.1f y=%.1f arcoErro=%.1f st=%u mot=%u\n",
                  static_cast<unsigned long>(nowMs - arcDiagnostic.startedMs), arcPhaseName(arcDiagnostic.phase),
                  f.valid ? f.distanceCm : -1.0f, l.valid ? l.distanceCm : -1.0f, r.valid ? r.distanceCm : -1.0f,
                  travel.leftMm, travel.rightMm,
                  motion.getLeftSpeedMmS(), motion.getLeftTargetMmS(),
                  motion.getRightSpeedMmS(), motion.getRightTargetMmS(),
                  motion.getTurnAngleDeg(), pose.xMm, pose.yMm,
                  arcDiagnostic.phase == ArcDiagnostic::Phase::Arc ? motion.getArcPathErrorMm() : 0.0f,
                  static_cast<unsigned>(motion.getStatus()), static_cast<unsigned>(motion.getFailureReason()));
}

void armArcDiagnostic(int dir) {
    if (!hardwareReady || !samplingEnabled.load() || ready || startDelay.pending() ||
        pivotDiagnostic.active() || arcDiagnostic.active()) {
        Serial.println("ARCO TESTE RECUSADO: robo ocupado, sensores pausados ou hardware indisponivel.");
        return;
    }
    motors.setEnabled(false);
    Serial.println("ARCO TESTE: calibrando IMU; mantenha o robo imovel, alinhado no corredor e de frente para a parede.");
    if (!imu.calibrateGyroZ()) { safeStop("falha na calibracao da IMU para o arco"); return; }
    arcDiagnostic = {};
    arcDiagnostic.dir = dir;
    arcDiagnostic.phase = ArcDiagnostic::Phase::Delay;
    arcDiagnostic.startedMs = millis();
    startDelay.arm(millis(), RunConfig::START_DELAY_MS);
    Serial.printf("ARCO TESTE ARMADO: curva a %s, raio %.0fmm, inicio com frente=%.1fcm; partida em 5s; x cancela.\n",
                  dir > 0 ? "esquerda" : "direita", ArcTest::RADIUS_MM, ArcTest::FRONT_TRIGGER_CM);
}

void finishArcDiagnostic(const char* outcome) {
    motion.stop();
    motors.brake();
    motors.setEnabled(false);
    delay(300);
    float f = NAN, l = NAN, r = NAN;
    measureAllIsolated(f, l, r);
    const auto pose = motion.getPose();
    const float lastSegmentDeg = motion.getTurnAngleDeg() * static_cast<float>(arcDiagnostic.dir);
    Serial.printf("ARCO RESULTADO %s: angulo_total=%.2fgraus recuos=%u eixo=(%+.1f,%+.1f)mm limite=%u aviso_trajeto=%u\n",
                  outcome, arcDiagnostic.doneDeg + lastSegmentDeg, static_cast<unsigned>(arcDiagnostic.recoveries),
                  pose.xMm, pose.yMm,
                  static_cast<unsigned>(motion.arcHardLimit()), static_cast<unsigned>(motion.arcPathWarning()));
    Serial.printf("ARCO SENSORES FINAIS (parado) F=%.2f L=%.2f R=%.2f cm\n", f, l, r);
    arcDiagnostic = {};
}

void updateArcDiagnostic() {
    using Phase = ArcDiagnostic::Phase;
    const uint32_t nowMs = millis();
    const float turnSide = static_cast<float>(arcDiagnostic.dir);

    if (arcDiagnostic.phase == Phase::Delay) {
        if (!startDelay.consumeIfDue(nowMs)) return;
        imu.resetYaw();
        if (!motion.begin(guidedMotionConfig(), {0.0f, 0.0f, 0.0f})) {
            safeStop("arco teste: falha ao iniciar encoders ou IMU");
            return;
        }
        arcDiagnostic.phase = Phase::Measure;
        arcDiagnostic.phaseMs = nowMs;
    }

    if (arcDiagnostic.phase == Phase::Measure) {
        float f = NAN, l = NAN, r = NAN;
        if (!measureAllIsolated(f, l, r)) { safeStop("arco teste: sonares nao pausaram"); return; }
        Serial.printf("ARCO MEDIDA (parado) F=%.2f L=%.2f R=%.2f cm; alvo F=%.1f\n", f, l, r, ArcTest::FRONT_TRIGGER_CM);
        if (!isfinite(f)) { safeStop("arco teste: sem parede na frente"); return; }
        const float targetCm = arcDiagnostic.sideConfirmed ? ArcTest::FRONT_TRIGGER_CM : ArcTest::SIDE_CHECK_FRONT_CM;
        const float errorCm = f - targetCm;
        if (!arcDiagnostic.sideConfirmed && fabsf(errorCm) <= ArcTest::FRONT_TOLERANCE_CM) {
            const float side = arcDiagnostic.dir > 0 ? l : r;
            if (isfinite(side) && side < ArcTest::SIDE_OPEN_MIN_CM) {
                safeStop("arco teste: lado da curva fechado");
                return;
            }
            arcDiagnostic.sideConfirmed = true;
            arcDiagnostic.corrections = 0;
            Serial.printf("ARCO LADO ABERTO confirmado (%.2fcm com frente=%.2fcm); recuando ate %.1fcm\n",
                          side, f, ArcTest::FRONT_TRIGGER_CM);
            return; // próxima medida mira o ponto de início do arco
        }
        if (arcDiagnostic.sideConfirmed && fabsf(errorCm) <= ArcTest::FRONT_TOLERANCE_CM) {
            motors.setEnabled(true);
            if (!motion.startRollingArc90(90.0f * turnSide, ArcTest::RADIUS_MM)) {
                safeStop("arco teste: falha ao iniciar o arco");
                return;
            }
            Serial.printf("ARCO INICIO: frente %.2fcm (erro %+.2fcm), raio %.0fmm\n", f, errorCm, ArcTest::RADIUS_MM);
            arcDiagnostic.phase = Phase::Arc;
            arcDiagnostic.phaseMs = nowMs;
            lastControlUpdateUs = micros();
            return;
        }
        if (++arcDiagnostic.corrections > ArcTest::MAX_CORRECTIONS + 4) {
            safeStop("arco teste: nao convergiu para a distancia de inicio");
            return;
        }
        const float moveMm = fmaxf(-150.0f, fminf(150.0f, errorCm * 10.0f));
        motors.setEnabled(true);
        if (!motion.startMoveLinear(moveMm)) { safeStop("arco teste: falha ao iniciar a aproximacao"); return; }
        Serial.printf("ARCO APROXIMAR: %+.1fmm (erro %+.2fcm)\n", moveMm, errorCm);
        arcDiagnostic.phase = Phase::Approach;
        arcDiagnostic.phaseMs = nowMs;
        lastControlUpdateUs = micros();
        return;
    }

    if (arcDiagnostic.phase == Phase::Settle) {
        logArcDebug(nowMs);
        if (nowMs - arcDiagnostic.phaseMs >= ArcTest::SETTLE_MS) {
            arcDiagnostic.phase = Phase::Measure;
            arcDiagnostic.phaseMs = nowMs;
        }
        return;
    }

    // Approach ou Arc: laço de controle a cada CONTROL_PERIOD_US.
    if (arcDiagnostic.phase == Phase::Approach && motion.isMovingForward()) {
        const auto front = getLateralSnapshot().frontEstimate(nowMs);
        const float approachTargetCm = arcDiagnostic.sideConfirmed ? ArcTest::FRONT_TRIGGER_CM : ArcTest::SIDE_CHECK_FRONT_CM;
        if (front.valid && front.distanceCm < approachTargetCm - 1.5f) motion.brakeForObstacle();
    }
    if (arcDiagnostic.phase == Phase::Exit)
        motion.setGuidedLateralSteering(true, motion.getDistanceTravelledMm() < POST_AVOID_MM ? postAvoidYawDeg : 0.0f);
    const uint32_t nowUs = micros();
    const uint32_t elapsedUs = nowUs - lastControlUpdateUs;
    if (elapsedUs < CONTROL_PERIOD_US) return;
    lastControlUpdateUs = nowUs;
    motion.update(static_cast<float>(elapsedUs) / 1000000.0f);
    logArcDebug(nowMs);

    const auto status = motion.getStatus();
    if (status == MotionStatus::Running) return;
    logArcDebug(nowMs, true);
    const auto reason = motion.getFailureReason();
    const bool contact = reason == MotionFailureReason::LeftWheelStalled ||
        reason == MotionFailureReason::RightWheelStalled || reason == MotionFailureReason::MotionTimeout;
    if (arcDiagnostic.phase == Phase::Arc && status != MotionStatus::Succeeded && contact &&
        arcDiagnostic.recoveries < ArcTest::MAX_RECOVERIES) {
        // Contato: guarda o ângulo já feito, recua um pouco e depois completa a curva.
        arcDiagnostic.doneDeg += motion.getTurnAngleDeg() * turnSide;
        ++arcDiagnostic.recoveries;
        motors.setEnabled(true);
        // Ré guiada: mantém o rumo atual (diagonal), até 18 mm/s.
        if (!motion.startGuidedRetreat(ArcTest::RECOVERY_RETREAT_MM)) {
            finishArcDiagnostic("FALHOU ao iniciar o recuo apos contato");
            return;
        }
        Serial.printf("ARCO CONTATO (motivo=%u) com %.1fgraus feitos: recuo %.0fmm, tentativa %u/%u\n",
                      static_cast<unsigned>(reason), arcDiagnostic.doneDeg, ArcTest::RECOVERY_RETREAT_MM,
                      static_cast<unsigned>(arcDiagnostic.recoveries), static_cast<unsigned>(ArcTest::MAX_RECOVERIES));
        arcDiagnostic.phase = Phase::Recover;
        arcDiagnostic.phaseMs = nowMs;
        lastControlUpdateUs = micros();
        return;
    }
    if (arcDiagnostic.phase == Phase::Recover && status == MotionStatus::Succeeded) {
        arcDiagnostic.doneDeg += motion.getTurnAngleDeg() * turnSide;
        const float remainingDeg = 90.0f - arcDiagnostic.doneDeg;
        if (remainingDeg < 5.0f) { finishArcDiagnostic("OK apos recuo"); return; }
        motors.setEnabled(true);
        if (!motion.startRollingArc90(remainingDeg * turnSide, ArcTest::RADIUS_MM)) {
            finishArcDiagnostic("FALHOU ao retomar o arco");
            return;
        }
        Serial.printf("ARCO RETOMADO: faltam %.1fgraus\n", remainingDeg);
        arcDiagnostic.phase = Phase::Arc;
        arcDiagnostic.phaseMs = nowMs;
        lastControlUpdateUs = micros();
        return;
    }
    if (status != MotionStatus::Succeeded) {
        char reason[64];
        snprintf(reason, sizeof(reason), "FALHOU na fase %s, motivo=%u",
                 arcPhaseName(arcDiagnostic.phase), static_cast<unsigned>(motion.getFailureReason()));
        finishArcDiagnostic(reason);
        return;
    }
    if (arcDiagnostic.phase == Phase::Exit) { postAvoidYawDeg = 0.0f; finishArcDiagnostic("OK (arco e saida de 10cm)"); return; }
    if (arcDiagnostic.phase == Phase::Arc) {
        // Igual à missão: reta de saída inclinada para longe da quina interna nos primeiros 40 mm.
        motion.shiftStraightReference(90.0f * turnSide);
        postAvoidYawDeg = -POST_AVOID_DEG * turnSide;
        motors.setEnabled(true);
        if (!motion.startMoveLinear(100.0f)) { finishArcDiagnostic("FALHOU ao iniciar a saida"); return; }
        Serial.printf("ARCO SAIDA: avancando 100mm, primeiros %.0fmm inclinado %+.0fgraus\n", POST_AVOID_MM, postAvoidYawDeg);
        arcDiagnostic.phase = Phase::Exit;
        arcDiagnostic.phaseMs = nowMs;
        lastControlUpdateUs = micros();
        return;
    }
    arcDiagnostic.phase = Phase::Settle;
    arcDiagnostic.phaseMs = nowMs;
}

void publishCurrentMapCell() {
    if (!telemetryClientReady || guidedRouteActive) return;
    char packet[400];
    const int length=mazeMap.cellPacketJson(mission.x(),mission.y(),mission.heading(),packet,sizeof(packet));
    if(length>0) telemetryClient.publishMapSnapshot(mission.x(),mission.y(),packet,length);
    routeRecorder.onTracking(robotStateEstimator.currentState().pose);
}

void finishMissionMotion(bool succeeded, bool frontExitConfirmed = false) {
    // Ré encurtada: logicamente ainda é uma célula inteira.
    if (succeeded && !missionMovingForward && missionBackCompMm != 0.0f)
        missionLinearProgressMm += missionBackCompMm;
    missionBackCompMm = 0.0f;
    if (succeeded && mission.state() == nav::MissionState::Moving) {
        const auto frame = getLateralSnapshot();
        const uint32_t now = millis();
        const auto front = frame.frontEstimate(now);
        const bool fresh = frame.ready && now - frame.sampledAtMs <= MAX_LATERAL_SAMPLE_AGE_MS;
        const float travelledMm = fabsf(missionLinearProgressMm);
        if ((missionMovingForward && missionLinearProgressMm <= 0.0f) ||
            (!missionMovingForward && missionLinearProgressMm >= 0.0f) ||
            !(FrontApproach::arrivalConfirmed(automaticFrontReferenceUsed, fresh && front.valid,
                                              front.distanceCm, travelledMm, mazeConfig.cellSizeMm) ||
              (missionMovingForward && automaticFrontReferenceUsed && travelledMm >= 0.5f*mazeConfig.cellSizeMm &&
               travelledMm <= mazeConfig.cellSizeMm + 20.0f))) {
            char cause[128];
            snprintf(cause, sizeof(cause), "chegada nao confirmada: percurso=%+.1fmm frente=%.2fcm celula=%umm",
                     missionLinearProgressMm, frame.front.distanceCm, mazeConfig.cellSizeMm);
            safeStop(cause);
            return;
        }
        if(automaticFrontReferenceUsed && !FrontApproach::confirmed(fresh && front.valid,front.distanceCm,travelledMm,mazeConfig.cellSizeMm))
            Serial.printf("CHEGADA AVISO: percurso=%+.1fmm aceito pela odometria; frente=%.2fcm divergente\n",
                missionLinearProgressMm,front.distanceCm);
        gridOdometry.reportArrival(missionLinearProgressMm);
        postAvoidYawDeg = 0.0f;
        if (missionMovingForward && sideSampleCount >= 2) {
            const bool leftOk = sideValidLeft >= 3, rightOk = sideValidRight >= 3;
            const uint16_t leftMm = leftOk ? medianMm(sideSamplesLeft, sideValidLeft) : 0;
            const uint16_t rightMm = rightOk ? medianMm(sideSamplesRight, sideValidRight) : 0;
            hardwareWallSensor.setSideOverride(leftOk, leftMm, rightOk, rightMm);
            Serial.printf("PAREDES LATERAIS medidas no centro da celula: E=%umm(%u validas) D=%umm(%u validas)%s\n",
                          static_cast<unsigned>(leftMm), static_cast<unsigned>(sideValidLeft),
                          static_cast<unsigned>(rightMm), static_cast<unsigned>(sideValidRight),
                          leftOk && rightOk ? "" : "; lado sem medida usa leitura ao vivo");
        }
        sideSampleCount = sideValidLeft = sideValidRight = 0;
        missionArrivalOffsetMm = missionMovingForward
            ? missionLinearProgressMm - static_cast<float>(mazeConfig.cellSizeMm) : 0.0f;
        // Com parede na frente, o sonar é mais confiável que o encoder (que conta o
        // escorregamento ao raspar a parede): sensor ~85 mm à frente do eixo e parede a
        // 85 mm do centro, então o eixo fica exatamente "frente" mm antes do centro.
        if (missionMovingForward && fresh && front.valid && isfinite(front.distanceCm) && front.distanceCm < 10.0f) {
            missionArrivalOffsetMm = -front.distanceCm * 10.0f;
            Serial.printf("CHEGADA PELA PAREDE: eixo %.1fmm antes do centro (frente=%.2fcm; encoder dizia %+.1fmm)\n",
                          -missionArrivalOffsetMm, front.distanceCm,
                          missionLinearProgressMm - static_cast<float>(mazeConfig.cellSizeMm));
        }
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
    // Depois de ré ou curva os sensores laterais (na frente do robô) olham a divisa com a
    // célula vizinha: se os lados desta célula já foram medidos, a observação usa o mapa.
    if (succeeded && !hardwareWallSensor.sideOverrideActive()) {
        const auto* cell = mazeMap.get(mission.x(), mission.y());
        const auto leftDir = mm::MazeMap::relToAbs(mission.heading(), mm::Rel::Left);
        const auto rightDir = mm::MazeMap::relToAbs(mission.heading(), mm::Rel::Right);
        if (cell && cell->isKnown(leftDir) && cell->isKnown(rightDir)) {
            const uint16_t leftMm = cell->hasWall(leftDir) ? 30 : 2000;
            const uint16_t rightMm = cell->hasWall(rightDir) ? 30 : 2000;
            hardwareWallSensor.setSideOverride(true, leftMm, true, rightMm);
            Serial.printf("PAREDES LATERAIS do mapa em (%u,%u): E=%s D=%s\n", mission.x(), mission.y(),
                          leftMm < 100 ? "parede" : "aberto", rightMm < 100 ? "parede" : "aberto");
        }
    }
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
            Serial.println("a=teste de curva em arco (raio 100mm) a esquerda; A=a direita; de frente para a parede; 5s de espera; x cancela.");
        } else if (command == 'v' || command == 'V') {
            armPivotDiagnostic(command == 'v' ? 90.0f : -90.0f);
        } else if (command == 'a' || command == 'A') {
            armArcDiagnostic(command == 'a' ? 1 : -1);
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

// Após confirmar posição/saída, recuo e arco usam somente IMU e encoders.
bool updateRollingCorner() {
    if(rollingPhase==RollingPhase::None)return false;
    const uint32_t now=millis();
    if(rollingPhase==RollingPhase::WaitSonarPause){
        if(!samplingPaused.load()){
            if(rollingSonarGate.pauseExpired(now))safeStop("curva: task nao confirmou a suspensao dos ultrassons");
            return true;
        }
        motors.setEnabled(true);
        if(!motion.startGuidedRetreat(rollingPlannedRetreatMm)){safeStop("curva: falha ao iniciar recuo");return true;}
        rollingPhase=RollingPhase::Retreat;lastControlUpdateUs=micros();return true;
    }
    if(rollingPhase==RollingPhase::BeforeArc){
        if(now-rollingPhaseMs<300)return true;
        motors.setEnabled(true);rollingPathWarningSent=false;rollingExitProgressMm=0;
        rollingRecoveries=0;rollingDoneDeg=0.0f;rollingArcStartPose=motion.getPose();
        if(!motion.startRollingArc90(90.0f*rollingDirection,rollingRadiusMm)){
            safeStop("curva: falha ao iniciar arco");return true;
        }
        rollingPhase=RollingPhase::Arc;lastControlUpdateUs=micros();return true;
    }
    if(rollingPhase==RollingPhase::Retreat || rollingPhase==RollingPhase::Arc ||
       rollingPhase==RollingPhase::RecoverRetreat){
        // Nenhum snapshot ultrassônico é consultado durante estes movimentos.
        const auto us=micros(),elapsed=us-lastControlUpdateUs;
        if(elapsed>=CONTROL_PERIOD_US){motion.update(elapsed/1000000.0f);lastControlUpdateUs=us;}
        if(rollingPhase==RollingPhase::Arc && motion.arcPathWarning() && !rollingPathWarningSent){
            rollingPathWarningSent=true;
            Serial.printf("ARCO AVISO: desvio do modelo %.1fmm; continuando pela IMU\n",motion.getArcPathErrorMm());
        }
        const float dirF=static_cast<float>(rollingDirection);
        const auto reason=motion.getFailureReason();
        const bool contact=reason==MotionFailureReason::LeftWheelStalled ||
            reason==MotionFailureReason::RightWheelStalled || reason==MotionFailureReason::MotionTimeout;
        if(motion.getStatus()==MotionStatus::Failed && rollingPhase==RollingPhase::Arc && contact &&
           rollingRecoveries<ArcTest::MAX_RECOVERIES){
            // Contato no arco: guarda o ângulo feito, recua mantendo o rumo e completa a curva.
            rollingDoneDeg+=motion.getTurnAngleDeg()*dirF;++rollingRecoveries;
            motors.setEnabled(true);
            if(!motion.startGuidedRetreat(ArcTest::RECOVERY_RETREAT_MM)){safeStop("curva: falha no recuo apos contato");return true;}
            Serial.printf("CURVA CONTATO (motivo=%u) com %.1fgraus: recuo %.0fmm, tentativa %u/%u\n",unsigned(reason),
                rollingDoneDeg,ArcTest::RECOVERY_RETREAT_MM,unsigned(rollingRecoveries),unsigned(ArcTest::MAX_RECOVERIES));
            rollingPhase=RollingPhase::RecoverRetreat;lastControlUpdateUs=micros();rollingPhaseMs=now;return true;
        }
        if(motion.getStatus()==MotionStatus::Succeeded && rollingPhase==RollingPhase::RecoverRetreat){
            rollingDoneDeg+=motion.getTurnAngleDeg()*dirF;
            const float remainingDeg=90.0f-rollingDoneDeg;
            if(remainingDeg>=5.0f){
                motors.setEnabled(true);
                // Contato costuma ser a quina interna: cada nova tentativa abre o arco 15 mm.
                const float retryRadiusMm=fminf(130.0f,rollingRadiusMm+15.0f*rollingRecoveries);
                if(!motion.startRollingArc90(remainingDeg*dirF,retryRadiusMm)){safeStop("curva: falha ao retomar arco");return true;}
                Serial.printf("CURVA RETOMADA: faltam %.1fgraus; raio %.0fmm\n",remainingDeg,retryRadiusMm);
                rollingPhase=RollingPhase::Arc;lastControlUpdateUs=micros();rollingPhaseMs=now;return true;
            }
            rollingPhase=RollingPhase::Arc; // ângulo já completo: segue para a conferência da saída
        }
        if(motion.getStatus()==MotionStatus::Failed){
            char cause[128];snprintf(cause,sizeof(cause),"curva: etapa=%u codigo=%u limite=%u giro=%.1f avanco=%.1f desvio=%.1fmm",
                unsigned(rollingPhase),unsigned(motion.getFailureReason()),unsigned(motion.arcHardLimit()),
                motion.getTurnAngleDeg(),motion.getDistanceTravelledMm(),motion.getArcPathErrorMm());
            safeStop(cause);return true;
        }
        if(motion.getStatus()!=MotionStatus::Succeeded)return true;
        if(rollingPhase==RollingPhase::Retreat){
            const float actual=motion.getDistanceTravelledMm();
            if(actual>=0 || fabsf(actual+rollingPlannedRetreatMm)>6){safeStop("curva: recuo fora da tolerancia");return true;}
            rollingRadiusMm=ArcTest::RADIUS_MM;
            rollingPhase=RollingPhase::BeforeArc;
            Serial.printf("CURVA: recuo confirmado %.1fmm; arco sem sonar, raio %.1fmm\n",-actual,rollingRadiusMm);
        }else{
            // Avanço na nova direção desde o início do primeiro trecho do arco (vale com recuos).
            const auto pose=motion.getPose();
            const float exitHeading=rollingArcStartPose.headingRad+dirF*MechanicalSettings::kPi*0.5f;
            rollingExitProgressMm=(pose.xMm-rollingArcStartPose.xMm)*cosf(exitHeading)+
                                  (pose.yMm-rollingArcStartPose.yMm)*sinf(exitHeading);
            if(!isfinite(rollingExitProgressMm) || rollingExitProgressMm<0 || rollingExitProgressMm>160){
                safeStop("arco: deslocamento de saida fora da faixa");return true;
            }
            motion.shiftStraightReference(90.0f*dirF);
            postAvoidYawDeg=-POST_AVOID_DEG*dirF;
            Serial.printf("ARCO CONCLUIDO: giro=%.1f; recuos=%u; avanco na nova direcao=%.1fmm\n",
                rollingDoneDeg+motion.getTurnAngleDeg()*dirF,unsigned(rollingRecoveries),rollingExitProgressMm);
            restoreRollingSonar();
            rollingPhase=RollingPhase::ConfirmExit;rollingExitVotes=0;rollingExitStamp=0;
        }
        rollingPhaseMs=now;return true;
    }
    // Antes de suspender e depois de retomar, só aceitar quadros novos e válidos.
    const auto frame=getLateralSnapshot();
    const bool fresh=frame.ready && now-frame.sampledAtMs<=MAX_LATERAL_SAMPLE_AGE_MS &&
        rollingSonarGate.accepts(frame.sampledAtMs);
    const auto f=frame.frontEstimate(now),l=frame.leftEstimate(now),rr=frame.rightEstimate(now);
    const bool valid=fresh && f.valid && l.valid && rr.valid && frame.front.valid && frame.left.valid && frame.right.valid;
    if(!valid){
        if(rollingPhase==RollingPhase::BeforeRetreat)rollingEntryGate.begin(now);
        if(!rollingPauseMs)rollingPauseMs=now;
        if(now-rollingPauseMs>2500)safeStop("curva: aguardando leituras novas antes/depois do arco por2500ms");
        return true;
    }
    rollingPauseMs=0;
    if(now-rollingPhaseMs<300)return true;
    if(rollingPhase==RollingPhase::BeforeRetreat){
        const auto exit=rollingDirection>0?l:rr;
        const auto& rawExit=rollingDirection>0?frame.left:frame.right;
        const bool geometryConfirmed=nav::missionCornerEligible(f.distanceCm,exit.distanceCm,mazeConfig.cellSizeMm) &&
            nav::missionCornerEligible(frame.front.distanceCm,rawExit.distanceCm,mazeConfig.cellSizeMm);
        rollingEntryGate.update(frame.sampledAtMs,geometryConfirmed,now);
        if(!rollingEntryGate.ready(now)){
            if(rollingEntryGate.expired(now))safeStop("curva: geometria persistentemente indefinida antes da suspensao");
            return true;
        }
        // Recua até a frente ler o ponto de início do arco aberto (~11 cm).
        rollingPlannedRetreatMm=(ArcTest::FRONT_TRIGGER_CM-f.distanceCm)*10.0f;
        if(!isfinite(rollingPlannedRetreatMm) || rollingPlannedRetreatMm<40.0f || rollingPlannedRetreatMm>100.0f){
            safeStop("curva: geometria de recuo invalida");return true;
        }
        rollingSonarGate.suspend(now);samplingEnabled.store(false);
        rollingPhase=RollingPhase::WaitSonarPause;
        Serial.printf("CURVA: suspender ultrassons; recuo %.1fmm; giro pela IMU e encoders\n",rollingPlannedRetreatMm);
        return true;
    }
    if(rollingPhase==RollingPhase::ConfirmExit){
        if(frame.sampledAtMs!=rollingExitStamp){
            rollingExitStamp=frame.sampledAtMs;
            // Depois do arco aberto o eixo já está ~10 mm dentro da célula seguinte: se ela
            // for um beco de uma célula a frente lê ~8 cm. Basta não estar diante da parede antiga.
            rollingExitVotes=isfinite(f.distanceCm) && f.distanceCm>5.0f?rollingExitVotes+1:0;
        }
        if(rollingExitVotes>=2){
            rollingPhase=RollingPhase::None;missionExitCreditMm=rollingExitProgressMm;
            gridOdometry.resetAccumulator();finishMissionMotion(true,true);
        }else if(now-rollingPhaseMs>2000)safeStop("curva: abertura frontal nao confirmada na saida");
    }
    return true;
}

void loop() {
    handleSerialCommands();
    handleRemoteCommands();
    if (pivotDiagnostic.active()) {
        updatePivotDiagnostic();
        delayMicroseconds(500);
        return;
    }
    if (arcDiagnostic.active()) {
        updateArcDiagnostic();
        delayMicroseconds(500);
        return;
    }
    const uint32_t nowMs = millis();
    const auto ageFrame=getLateralSnapshot();
    if(ageFrame.ready && rollingSonarGate.accepts(ageFrame.sampledAtMs)) {
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
        // Crédito negativo: o primeiro avanço (se for o primeiro movimento) anda a mais.
        missionExitCreditMm = -RunConfig::START_BEHIND_CENTER_MM;
        Serial.printf("LARGADA: eixo %.0fmm atras do centro; primeiro avanco compensa\n", RunConfig::START_BEHIND_CENTER_MM);
        missionResumeCount = 0;
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

    {
        // Debug da missão a 10 Hz: célula, fase da curva, sonares, encoders, IMU e status.
        static uint32_t lastMissionDebugMs = 0;
        if (nowMs - lastMissionDebugMs >= 100) {
            lastMissionDebugMs = nowMs;
            const auto frame = getLateralSnapshot();
            const auto f = frame.frontEstimate(nowMs), l = frame.leftEstimate(nowMs), r = frame.rightEstimate(nowMs);
            const auto travel = motion.getWheelTravel();
            const auto pose = motion.getPose();
            Serial.printf("MDBG t=%lu cel=(%u,%u) rumo=%u est=%u curva=%u F=%.1f L=%.1f R=%.1f "
                          "encE=%.1f encD=%.1f velE=%.0f/%.0f velD=%.0f/%.0f ang=%.2f x=%.1f y=%.1f st=%u mot=%u\n",
                          static_cast<unsigned long>(nowMs - attemptStartedMs), mission.x(), mission.y(),
                          static_cast<unsigned>(mission.heading()), static_cast<unsigned>(mission.state()),
                          static_cast<unsigned>(rollingPhase),
                          f.valid ? f.distanceCm : -1.0f, l.valid ? l.distanceCm : -1.0f, r.valid ? r.distanceCm : -1.0f,
                          travel.leftMm, travel.rightMm, motion.getLeftSpeedMmS(), motion.getLeftTargetMmS(),
                          motion.getRightSpeedMmS(), motion.getRightTargetMmS(), motion.getTurnAngleDeg(),
                          pose.xMm, pose.yMm, static_cast<unsigned>(motion.getStatus()),
                          static_cast<unsigned>(motion.getFailureReason()));
        }
    }
    if(updateRollingCorner()) {delayMicroseconds(500);return;}

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
            if (automaticForward && !stale && snapshot.sampledAtMs != lastSideSampleStamp &&
                sideSampleCount < SIDE_SAMPLES_MAX) {
                const float axleMm = missionLinearProgressMm + motion.getDistanceTravelledMm();
                const float sensorAtCentreMm = mazeConfig.cellSizeMm - SIDE_SENSOR_AHEAD_MM;
                if (fabsf(axleMm - sensorAtCentreMm) <= 20.0f) {
                    lastSideSampleStamp = snapshot.sampledAtMs;
                    const auto sl = snapshot.leftEstimate(sensorNowMs), sr = snapshot.rightEstimate(sensorNowMs);
                    // Só leituras válidas: sem eco não prova abertura (labirinto, 23/11).
                    if (sl.valid) sideSamplesLeft[sideValidLeft++] = static_cast<uint16_t>(sl.distanceCm * 10.0f);
                    if (sr.valid) sideSamplesRight[sideValidRight++] = static_cast<uint16_t>(sr.distanceCm * 10.0f);
                    ++sideSampleCount;
                }
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
            const auto straightFeedback=automaticForward ? straightProximity.update(snapshot) : StraightProximityGuard::Feedback{};
            if(automaticForward){
                if(frontBlocked){
                    char cause[128];snprintf(cause,sizeof(cause),"reta: proximidade confirmada F=%u E=%u D=%u; sem recuo automatico",
                        frontBlocked,straightFeedback.leftConfirmed,straightFeedback.rightConfirmed);
                    safeStop(cause);return;
                }
                const auto lateral=guidedLateralClearance.update(snapshot);
                if(straightFeedback.leftConfirmed || straightFeedback.rightConfirmed){
                    // Parede lateral perto demais: em vez de parar, segue devagar com o
                    // desvio máximo para o lado oposto (positivo = esquerda).
                    const float away=straightFeedback.rightConfirmed && !straightFeedback.leftConfirmed ? 10.0f :
                                     straightFeedback.leftConfirmed && !straightFeedback.rightConfirmed ? -10.0f : 0.0f;
                    motion.setGuidedLateralSteering(true,away);
                    static uint32_t lastLateralNoticeMs=0;
                    if(millis()-lastLateralNoticeMs>1000){
                        lastLateralNoticeMs=millis();
                        Serial.printf("DESVIO LATERAL: E=%.2f D=%.2fcm; desviando %+.0fgraus e seguindo devagar\n",
                                      snapshot.left.distanceCm,snapshot.right.distanceCm,away);
                    }
                } else if (postAvoidYawDeg != 0.0f && motion.getDistanceTravelledMm() < POST_AVOID_MM) {
                    motion.setGuidedLateralSteering(true,postAvoidYawDeg);
                } else {
                    motion.setGuidedLateralSteering(lateral.slow,lateral.yawOffsetDeg);
                }
            }
            if (guidedForward && cornerSequenceActive() && guidedRecovery.phase != RecoveryPhase::Braking) {
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
                const auto decision=frontEchoRecovery.update(!frontInvalid && !rightInvalidDuringShift && !retreatFrontInvalid && !straightFeedback.pause,
                    snapshot.sampledAtMs,millis(),
                    motion.sensorPauseSettled(),automaticForward);
                if(decision==FrontEchoRecovery::Decision::Failed) {
                    mission.fail();
                    safeStop(retreating ? "recuperacao guiada: eco frontal invalido ou encurtando durante a re"
                             : shifting ? "curva: eco frontal ou direito instavel durante a diagonal"
                                      : "sonar instavel: confirmacao excedeu o limite de recuperacao; sem recuo na reta");
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
                        : (hold ? "SONAR: freando e aguardando novas leituras; sem recuo na reta"
                                : "SONAR: duas leituras validas; retomando o restante do avanco"));
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
                // Com parede na frente o eixo não alcança o centro: depois de meia célula,
                // a própria parede é a referência de chegada (o atraso vai para a próxima ré).
                if(advanceMm < 0.5f*mazeConfig.cellSizeMm) {
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
        if (missionAlignPending) {
            // Alinhamento antes da ré concluído (ou não): segue com a ré planejada.
            missionAlignPending = false;
            Serial.printf("ALINHADO (status=%u, giro=%.2fgraus); re de %.1fmm\n",
                          static_cast<unsigned>(status), motion.getTurnAngleDeg(), missionPendingBackMm);
            motors.setEnabled(true);
            if (!motion.startMoveLinear(-missionPendingBackMm)) { safeStop("falha ao iniciar a re apos alinhar"); return; }
            trackStartedMotion();
            return;
        }
        if (!sequence && linear && missionMovingForward) {
            // Recuperação da reta da missão: contato ou roda presa -> ré curta e retoma o resto.
            const auto reason = motion.getFailureReason();
            const float travelledMm = motion.getDistanceTravelledMm();
            if (missionRetreatPending) {
                missionRetreatPending = false;
                missionLinearProgressMm += travelledMm;
                if (status != MotionStatus::Succeeded) { safeStop("reta: re de recuperacao falhou"); return; }
                // A ré pode deixar o eixo atrás do início da célula: um avanço aceita no máximo uma célula.
                const float remainingMm = fminf(static_cast<float>(mazeConfig.cellSizeMm),
                    static_cast<float>(mazeConfig.cellSizeMm) - missionLinearProgressMm);
                if (remainingMm < 10.0f) { finishMissionMotion(true); return; }
                motors.setEnabled(true);
                straightProximity.reset();
                motion.setFrontApproachMode(false);
                if (!motion.startMoveLinear(remainingMm)) { safeStop("reta: falha ao retomar o avanco"); return; }
                Serial.printf("RETA RETOMADA: faltam %.1fmm da celula\n", remainingMm);
                trackStartedMotion();
                return;
            }
            const bool recoverable = status == MotionStatus::Failed &&
                (reason == MotionFailureReason::LeftWheelStalled || reason == MotionFailureReason::RightWheelStalled ||
                 reason == MotionFailureReason::Obstacle || reason == MotionFailureReason::MotionTimeout ||
                 reason == MotionFailureReason::SafetyLimit);
            if (recoverable && missionStraightRetries < 3) {
                ++missionStraightRetries;
                missionLinearProgressMm += travelledMm;
                if (missionLinearProgressMm < 20.0f) {
                    // Junto da parede de largada não há espaço para ré: tenta de novo para frente.
                    const float remainingMm = fminf(static_cast<float>(mazeConfig.cellSizeMm) + 60.0f,
                        static_cast<float>(mazeConfig.cellSizeMm) - missionLinearProgressMm);
                    motors.setEnabled(true);
                    straightProximity.reset();
                    if (!motion.startMoveLinear(remainingMm)) { safeStop("reta: falha ao repetir o avanco"); return; }
                    Serial.printf("RETA TRAVOU (motivo=%u) junto da largada: tenta de novo sem re (%u/3)\n",
                                  static_cast<unsigned>(reason), static_cast<unsigned>(missionStraightRetries));
                    trackStartedMotion();
                    return;
                }
                motors.setEnabled(true);
                if (!motion.startGuidedRetreat(15.0f)) { safeStop("reta: falha ao iniciar re de recuperacao"); return; }
                missionRetreatPending = true;
                Serial.printf("RETA TRAVOU (motivo=%u) apos %.1fmm: re 15mm e retoma; tentativa %u/3\n",
                              static_cast<unsigned>(reason), missionLinearProgressMm,
                              static_cast<unsigned>(missionStraightRetries));
                trackStartedMotion();
                return;
            }
        }
        const bool recoverableStop = status == MotionStatus::Failed && linear && sequence &&
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
