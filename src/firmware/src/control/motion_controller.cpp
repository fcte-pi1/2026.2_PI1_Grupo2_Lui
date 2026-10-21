#include "motion_controller.hpp"
#include <Arduino.h>

#include <math.h>

namespace {
constexpr float kPiRadians = 3.14159265358979323846f;

float absValue(float value) { return value < 0.0f ? -value : value; }

float clampValue(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

float signOf(float value) {
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

float approach(float current, float target, float maxDelta) {
    const float difference = target - current;
    if (difference > maxDelta) return current + maxDelta;
    if (difference < -maxDelta) return current - maxDelta;
    return target;
}

float wrapDegrees(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}
}

MotionControllerConfig::MotionControllerConfig()
    : leftCountsPerMm(6.74f),
      rightCountsPerMm(6.74f),
      wheelTrackMm(90.0f),
      cellSizeMm(0.0f),
      maxLinearSpeedMmS(100.0f),
      minLinearSpeedMmS(20.0f),
      linearAccelerationMmS2(250.0f),
      maxAngularSpeedDegS(75.0f),
      minAngularSpeedDegS(15.0f),
      angularAccelerationDegS2(240.0f),
      turnKpDegPerSecPerDeg(2.0f),
      positionToleranceMm(4.0f),
      angleToleranceDeg(2.5f),
      settledSpeedMmS(8.0f),
      settledGyroDegS(5.0f),
      straightCorrectionKp(1.0f),
      wallAlignmentKp(1.0f),
      leftFeedForwardPwmPerMmS(0.8f),
      rightFeedForwardPwmPerMmS(0.8f),
      leftStaticFrictionPwm(20.0f),
      rightStaticFrictionPwm(20.0f),
      imuYawSign(1.0f),
      forwardTimeoutMs(5000),
      turnTimeoutMs(3500),
      stallTimeoutMs(500),
      stallMinSpeedMmS(25.0f) {
}

MotionController::MotionController(MotorDRV8833& motors,
                                   EncoderN20& leftEncoder,
                                   EncoderN20& rightEncoder,
                                   IMUMPU6500& imu,
                                   Odometry& odometry,
                                   PIDController& leftPid,
                                   PIDController& rightPid)
    : motors_(motors), leftEncoder_(leftEncoder),
      rightEncoder_(rightEncoder), imu_(imu), odometry_(odometry),
      leftPid_(leftPid), rightPid_(rightPid) {
}

bool MotionController::begin(const MotionControllerConfig& config,
                             const Pose2D& initialPose) {
    if (config.leftCountsPerMm <= 0.0f ||
        config.rightCountsPerMm <= 0.0f ||
        config.wheelTrackMm <= 0.0f ||
        config.cellSizeMm <= 0.0f ||
        config.maxLinearSpeedMmS <= 0.0f ||
        config.minLinearSpeedMmS <= 0.0f ||
        config.minLinearSpeedMmS > config.maxLinearSpeedMmS ||
        config.linearAccelerationMmS2 <= 0.0f ||
        config.maxAngularSpeedDegS <= 0.0f ||
        config.minAngularSpeedDegS <= 0.0f ||
        config.minAngularSpeedDegS > config.maxAngularSpeedDegS ||
        config.angularAccelerationDegS2 <= 0.0f ||
        config.turnKpDegPerSecPerDeg <= 0.0f ||
        config.straightCorrectionKp < 0.0f ||
        config.wallAlignmentKp < 0.0f ||
        config.positionToleranceMm <= 0.0f ||
        config.angleToleranceDeg <= 0.0f ||
        config.imuYawSign == 0.0f ||
        config.forwardTimeoutMs == 0 || config.turnTimeoutMs == 0 ||
        config.stallTimeoutMs == 0) {
        return false;
    }

    if (!leftEncoder_.isInitialized() || !rightEncoder_.isInitialized()) {
        return false;
    }

    int32_t leftCount = 0;
    int32_t rightCount = 0;
    if (!leftEncoder_.readCount(leftCount) ||
        !rightEncoder_.readCount(rightCount)) {
        return false;
    }

    OdometryConfig odometryConfig;
    odometryConfig.leftCountsPerMm = config.leftCountsPerMm;
    odometryConfig.rightCountsPerMm = config.rightCountsPerMm;
    odometryConfig.wheelTrackMm = config.wheelTrackMm;

    if (!odometry_.begin(odometryConfig, initialPose, leftCount, rightCount)) {
        return false;
    }

    config_ = config;
    previousLeftCount_ = leftCount;
    previousRightCount_ = rightCount;
    initialized_ = true;
    status_ = MotionStatus::Idle;
    motionType_ = MotionType::None;
    failureReason_ = MotionFailureReason::None;
    return true;
}

bool MotionController::prepareMotion(MotionType type, uint32_t timeoutMs) {
    if (!initialized_) {
        failureReason_ = MotionFailureReason::NotInitialized;
        status_ = MotionStatus::Failed;
        return false;
    }
    if (status_ == MotionStatus::Running) {
        // Não interromper/substituir uma operação ainda em andamento.
        return false;
    }
    if (!motors_.isEnabled()) {
        failureReason_ = MotionFailureReason::MotorsDisabled;
        status_ = MotionStatus::Failed;
        motionType_ = MotionType::None;
        motors_.stop();
        return false;
    }

    int32_t leftCount = 0;
    int32_t rightCount = 0;
    if (!leftEncoder_.readCount(leftCount) ||
        !rightEncoder_.readCount(rightCount)) {
        fail(MotionFailureReason::EncoderReadFailed);
        return false;
    }

    startLeftCount_ = leftCount;
    startRightCount_ = rightCount;
    previousLeftCount_ = leftCount;
    previousRightCount_ = rightCount;

    motionStartedMs_ = millis();
    motionTimeoutMs_ = timeoutMs;
    lastLeftProgressMs_ = motionStartedMs_;
    lastRightProgressMs_ = motionStartedMs_;
    settledSamples_ = 0;

    profiledLinearSpeedMmS_ = 0.0f;
    profiledAngularSpeedDegS_ = 0.0f;
    wallAlignmentValid_ = false;
    lastLeftTargetMmS_ = 0.0f;
    lastRightTargetMmS_ = 0.0f;
    distanceTravelledMm_ = 0.0f;
    turnAngleDeg_ = 0.0f;

    leftPid_.reset();
    rightPid_.reset();

    motionType_ = type;
    failureReason_ = MotionFailureReason::None;
    status_ = MotionStatus::Running;
    return true;
}

bool MotionController::startMoveForwardOneCell() {
    return startMoveLinear(config_.cellSizeMm);
}

bool MotionController::startMoveBackwardOneCell() {
    return startMoveLinear(-config_.cellSizeMm);
}

bool MotionController::startMoveLinear(float signedDistanceMm) {
    // Como as demais ações, não substitui uma operação que ainda está rodando.
    if (status_ == MotionStatus::Running) return false;
    if (!isfinite(signedDistanceMm) || absValue(signedDistanceMm) < 0.5f) {
        failureReason_ = MotionFailureReason::InvalidDistance;
        if (status_ != MotionStatus::Running) status_ = MotionStatus::Failed;
        motors_.stop();
        return false;
    }
    if (!prepareMotion(MotionType::ForwardCell, config_.forwardTimeoutMs)) {
        return false;
    }
    targetDistanceMm_ = signedDistanceMm;
    return true;
}

void MotionController::setWallAlignmentFeedback(float leftDistanceMm,
                                                float rightDistanceMm,
                                                bool valid) {
    wallAlignmentValid_ = valid && isfinite(leftDistanceMm) &&
        isfinite(rightDistanceMm) && leftDistanceMm > 0.0f &&
        rightDistanceMm > 0.0f;
    if (wallAlignmentValid_) {
        leftWallDistanceMm_ = leftDistanceMm;
        rightWallDistanceMm_ = rightDistanceMm;
    }
}

bool MotionController::startTurn(float targetAngleDeg, MotionType type) {
    if (!initialized_) {
        failureReason_ = MotionFailureReason::NotInitialized;
        status_ = MotionStatus::Failed;
        motors_.stop();
        return false;
    }
    if (!imu_.isInitialized()) {
        fail(MotionFailureReason::ImuReadFailed);
        return false;
    }
    if (!prepareMotion(type, config_.turnTimeoutMs)) return false;

    MPU6500Reading reading{};
    if (!imu_.read(reading)) {
        fail(MotionFailureReason::ImuReadFailed);
        return false;
    }

    targetAngleDeg_ = targetAngleDeg;
    turnAngleDeg_ = 0.0f;
    lastImuYawDeg_ = reading.yawDegrees;
    turnStartHeadingRad_ = odometry_.getPose().headingRad;
    return true;
}

bool MotionController::startTurnLeft90() {
    return startTurn(90.0f, MotionType::TurnLeft90);
}

bool MotionController::startTurnRight90() {
    return startTurn(-90.0f, MotionType::TurnRight90);
}

bool MotionController::startTurnAround180() {
    // Escolha determinística: meia-volta para a esquerda.
    return startTurn(180.0f, MotionType::TurnAround180);
}

void MotionController::update(float dtSeconds) {
    if (status_ != MotionStatus::Running) return;

    if (dtSeconds <= 0.0f || dtSeconds > 0.1f || !isfinite(dtSeconds)) {
        fail(MotionFailureReason::InvalidTimeStep);
        return;
    }

    int32_t leftCount = 0;
    int32_t rightCount = 0;
    if (!leftEncoder_.readCount(leftCount) ||
        !rightEncoder_.readCount(rightCount)) {
        fail(MotionFailureReason::EncoderReadFailed);
        return;
    }

    const int32_t deltaLeftCount = leftCount - previousLeftCount_;
    const int32_t deltaRightCount = rightCount - previousRightCount_;

    const float leftDeltaMm =
        static_cast<float>(deltaLeftCount) / config_.leftCountsPerMm;
    const float rightDeltaMm =
        static_cast<float>(deltaRightCount) / config_.rightCountsPerMm;
    const float leftSpeedMmS = leftDeltaMm / dtSeconds;
    const float rightSpeedMmS = rightDeltaMm / dtSeconds;

    previousLeftCount_ = leftCount;
    previousRightCount_ = rightCount;

    if (!odometry_.update(leftCount, rightCount)) {
        fail(MotionFailureReason::EncoderReadFailed);
        return;
    }

    const uint32_t now = millis();
    updateStallTimers(deltaLeftCount, deltaRightCount, now);

    if (motionType_ == MotionType::ForwardCell) {
        updateForward(dtSeconds, leftCount, rightCount,
                      leftSpeedMmS, rightSpeedMmS, now);
    } else if (motionType_ == MotionType::TurnLeft90 ||
               motionType_ == MotionType::TurnRight90 ||
               motionType_ == MotionType::TurnAround180) {
        updateTurn(dtSeconds, leftSpeedMmS, rightSpeedMmS, now);
    } else {
        fail(MotionFailureReason::Cancelled);
    }
}

void MotionController::updateForward(float dtSeconds,
                                     int32_t leftCount,
                                     int32_t rightCount,
                                     float leftSpeedMmS,
                                     float rightSpeedMmS,
                                     uint32_t nowMs) {
    const float leftTravelMm =
        static_cast<float>(leftCount - startLeftCount_) /
        config_.leftCountsPerMm;
    const float rightTravelMm =
        static_cast<float>(rightCount - startRightCount_) /
        config_.rightCountsPerMm;
    distanceTravelledMm_ = (leftTravelMm + rightTravelMm) / 2.0f;

    const float remainingMm = targetDistanceMm_ - distanceTravelledMm_;
    float desiredSpeed = 0.0f;

    if (absValue(remainingMm) > 0.5f) {
        const float stoppingSpeed = sqrtf(
            2.0f * config_.linearAccelerationMmS2 * absValue(remainingMm));
        float magnitude = stoppingSpeed < config_.maxLinearSpeedMmS
            ? stoppingSpeed : config_.maxLinearSpeedMmS;
        // Não impõe velocidade mínima perto do alvo, para permitir frenagem suave.
        if (absValue(remainingMm) > config_.positionToleranceMm * 2.0f &&
            magnitude < config_.minLinearSpeedMmS) {
            magnitude = config_.minLinearSpeedMmS;
        }
        desiredSpeed = signOf(remainingMm) * magnitude;
    }

    profiledLinearSpeedMmS_ = approach(
        profiledLinearSpeedMmS_, desiredSpeed,
        config_.linearAccelerationMmS2 * dtSeconds);

    // Se a roda esquerda avançou mais que a direita, reduz a referência da
    // esquerda e aumenta a da direita. A correção some quando a velocidade zera.
    float correction = config_.straightCorrectionKp *
                       (leftTravelMm - rightTravelMm);
    if (wallAlignmentValid_) {
        // Diferença positiva significa que há mais espaço à esquerda; para
        // alinhar, a correção muda de sinal quando o robô está em marcha à ré.
        float wallCorrection = config_.wallAlignmentKp *
            (leftWallDistanceMm_ - rightWallDistanceMm_);
        if (profiledLinearSpeedMmS_ < 0.0f) wallCorrection = -wallCorrection;
        correction += wallCorrection;
    }
    const float correctionLimit = config_.maxLinearSpeedMmS * 0.35f;
    correction = clampValue(correction, -correctionLimit, correctionLimit);
    if (absValue(profiledLinearSpeedMmS_) < config_.settledSpeedMmS) {
        correction = 0.0f;
    } else {
        const float scale = absValue(profiledLinearSpeedMmS_) /
                            config_.maxLinearSpeedMmS;
        correction *= scale > 1.0f ? 1.0f : scale;
    }

    float leftTarget = profiledLinearSpeedMmS_ - correction;
    float rightTarget = profiledLinearSpeedMmS_ + correction;
    leftTarget = clampValue(leftTarget, -config_.maxLinearSpeedMmS,
                            config_.maxLinearSpeedMmS);
    rightTarget = clampValue(rightTarget, -config_.maxLinearSpeedMmS,
                             config_.maxLinearSpeedMmS);

    applyWheelSpeedTargets(leftTarget, rightTarget,
                           leftSpeedMmS, rightSpeedMmS, dtSeconds);

    if (absValue(remainingMm) <= config_.positionToleranceMm &&
        absValue(leftSpeedMmS) <= config_.settledSpeedMmS &&
        absValue(rightSpeedMmS) <= config_.settledSpeedMmS &&
        absValue(profiledLinearSpeedMmS_) <= config_.settledSpeedMmS) {
        if (++settledSamples_ >= 3) {
            succeed();
            return;
        }
    } else {
        settledSamples_ = 0;
    }

    if (checkTimeoutAndStall(nowMs, leftTarget, rightTarget)) return;
}

void MotionController::updateTurn(float dtSeconds,
                                  float leftSpeedMmS,
                                  float rightSpeedMmS,
                                  uint32_t nowMs) {
    MPU6500Reading reading{};
    if (!imu_.read(reading)) {
        fail(MotionFailureReason::ImuReadFailed);
        return;
    }

    const float wrappedDelta = wrapDegrees(reading.yawDegrees - lastImuYawDeg_);
    turnAngleDeg_ += wrappedDelta * config_.imuYawSign;
    lastImuYawDeg_ = reading.yawDegrees;

    const float measuredGyroDegS = reading.gyroZdps * config_.imuYawSign;
    const float errorDeg = targetAngleDeg_ - turnAngleDeg_;
    float desiredAngularSpeed = 0.0f;

    if (absValue(errorDeg) > 0.5f) {
        // Limite de velocidade baseado na distância angular restante para frear.
        const float stoppingRate = sqrtf(
            2.0f * config_.angularAccelerationDegS2 * absValue(errorDeg));
        float magnitude = stoppingRate < config_.maxAngularSpeedDegS
            ? stoppingRate : config_.maxAngularSpeedDegS;

        // O termo proporcional impede uma velocidade excessiva perto do alvo.
        const float proportionalLimit =
            config_.turnKpDegPerSecPerDeg * absValue(errorDeg);
        if (magnitude > proportionalLimit) {
            magnitude = proportionalLimit;
        }
        // Só impõe velocidade mínima quando ainda está longe do alvo.
        if (absValue(errorDeg) > config_.angleToleranceDeg * 2.0f &&
            magnitude < config_.minAngularSpeedDegS) {
            magnitude = config_.minAngularSpeedDegS;
        }
        desiredAngularSpeed = signOf(errorDeg) * magnitude;
    }

    profiledAngularSpeedDegS_ = approach(
        profiledAngularSpeedDegS_, desiredAngularSpeed,
        config_.angularAccelerationDegS2 * dtSeconds);

    // Convenção: omega positivo = giro anti-horário/esquerda.
    // dR - dL = omega * wheelTrack. Portanto a roda esquerda vai para trás
    // e a direita para frente durante um giro positivo no próprio eixo.
    const float omegaRadS = profiledAngularSpeedDegS_ * kPiRadians / 180.0f;
    const float leftTarget = -omegaRadS * config_.wheelTrackMm / 2.0f;
    const float rightTarget = omegaRadS * config_.wheelTrackMm / 2.0f;

    applyWheelSpeedTargets(leftTarget, rightTarget,
                           leftSpeedMmS, rightSpeedMmS, dtSeconds);

    if (absValue(errorDeg) <= config_.angleToleranceDeg &&
        absValue(measuredGyroDegS) <= config_.settledGyroDegS &&
        absValue(leftSpeedMmS) <= config_.settledSpeedMmS &&
        absValue(rightSpeedMmS) <= config_.settledSpeedMmS &&
        absValue(profiledAngularSpeedDegS_) <= config_.settledGyroDegS) {
        if (++settledSamples_ >= 3) {
            // Corrige a orientação da odometria com o ângulo relativo medido
            // pela IMU durante este giro. x/y continuam baseados nos encoders.
            odometry_.setHeadingRad(
                turnStartHeadingRad_ + turnAngleDeg_ * kPiRadians / 180.0f);
            succeed();
            return;
        }
    } else {
        settledSamples_ = 0;
    }

    if (checkTimeoutAndStall(nowMs, leftTarget, rightTarget)) return;
}

void MotionController::applyWheelSpeedTargets(float leftTargetMmS,
                                              float rightTargetMmS,
                                              float leftMeasuredMmS,
                                              float rightMeasuredMmS,
                                              float dtSeconds) {
    lastLeftTargetMmS_ = leftTargetMmS;
    lastRightTargetMmS_ = rightTargetMmS;

    const float leftCorrection = leftPid_.update(
        leftTargetMmS, leftMeasuredMmS, dtSeconds);
    const float rightCorrection = rightPid_.update(
        rightTargetMmS, rightMeasuredMmS, dtSeconds);

    // Feed-forward fornece a atuação necessária em regime permanente; o PID
    // corrige o erro. Sem feed-forward, uma saída PID puramente baseada no
    // erro tende a cair para zero quando a velocidade chega ao setpoint.
    float leftFeedForward = 0.0f;
    if (absValue(leftTargetMmS) > 0.5f) {
        leftFeedForward = config_.leftFeedForwardPwmPerMmS * leftTargetMmS +
            signOf(leftTargetMmS) * config_.leftStaticFrictionPwm;
    }
    float rightFeedForward = 0.0f;
    if (absValue(rightTargetMmS) > 0.5f) {
        rightFeedForward = config_.rightFeedForwardPwmPerMmS * rightTargetMmS +
            signOf(rightTargetMmS) * config_.rightStaticFrictionPwm;
    }

    const float leftOutput = leftFeedForward + leftCorrection;
    const float rightOutput = rightFeedForward + rightCorrection;

    const int16_t leftPwm = static_cast<int16_t>(
        clampValue(leftOutput, -255.0f, 255.0f));
    const int16_t rightPwm = static_cast<int16_t>(
        clampValue(rightOutput, -255.0f, 255.0f));

    motors_.setSpeeds(leftPwm, rightPwm);
}

void MotionController::updateStallTimers(int32_t leftDeltaCount,
                                         int32_t rightDeltaCount,
                                         uint32_t nowMs) {
    if (leftDeltaCount != 0) lastLeftProgressMs_ = nowMs;
    if (rightDeltaCount != 0) lastRightProgressMs_ = nowMs;
}

bool MotionController::checkTimeoutAndStall(uint32_t nowMs,
                                            float leftTargetMmS,
                                            float rightTargetMmS) {
    if (static_cast<uint32_t>(nowMs - motionStartedMs_) >= motionTimeoutMs_) {
        fail(MotionFailureReason::MotionTimeout);
        return true;
    }

    if (absValue(leftTargetMmS) >= config_.stallMinSpeedMmS &&
        static_cast<uint32_t>(nowMs - lastLeftProgressMs_) >= config_.stallTimeoutMs) {
        fail(MotionFailureReason::LeftWheelStalled);
        return true;
    }

    if (absValue(rightTargetMmS) >= config_.stallMinSpeedMmS &&
        static_cast<uint32_t>(nowMs - lastRightProgressMs_) >= config_.stallTimeoutMs) {
        fail(MotionFailureReason::RightWheelStalled);
        return true;
    }

    return false;
}

void MotionController::fail(MotionFailureReason reason) {
    // Falha/cancelamento exige rearmamento explícito antes de novo movimento.
    motors_.stop();
    motors_.setEnabled(false);
    leftPid_.reset();
    rightPid_.reset();
    status_ = MotionStatus::Failed;
    // Preserva motionType_ para diagnóstico do movimento que falhou.
    failureReason_ = reason;
    profiledLinearSpeedMmS_ = 0.0f;
    profiledAngularSpeedDegS_ = 0.0f;
}

void MotionController::succeed() {
    motors_.stop();
    leftPid_.reset();
    rightPid_.reset();
    status_ = MotionStatus::Succeeded;
    // Preserva motionType_ para que o chamador saiba qual operação terminou.
    failureReason_ = MotionFailureReason::None;
    profiledLinearSpeedMmS_ = 0.0f;
    profiledAngularSpeedDegS_ = 0.0f;
}

void MotionController::stop() {
    fail(MotionFailureReason::Cancelled);
}

MotionStatus MotionController::getStatus() const { return status_; }
MotionType MotionController::getMotionType() const { return motionType_; }
MotionFailureReason MotionController::getFailureReason() const { return failureReason_; }
Pose2D MotionController::getPose() const { return odometry_.getPose(); }
float MotionController::getDistanceTravelledMm() const { return distanceTravelledMm_; }
float MotionController::getTurnAngleDeg() const { return turnAngleDeg_; }
