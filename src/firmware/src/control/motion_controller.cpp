#include "motion_controller.hpp"
#include "../config/mechanical_config.hpp"
#include "front_approach_policy.hpp"
#include <Arduino.h>
#include <math.h>

namespace {
constexpr float pi = 3.14159265358979323846f;
constexpr float guidedPivotToleranceDeg = 1.2f;
constexpr float guidedPivotMaxDriftMm = 6.0f;
float clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
float sign(float x) { return x < 0 ? -1.0f : 1.0f; }
float wrap(float x) { while (x >= 180) x -= 360; while (x < -180) x += 360; return x; }
}

MotionControllerConfig::MotionControllerConfig()
    : leftCountsPerMm(MechanicalSettings::LEFT_COUNTS_PER_MM),
      rightCountsPerMm(MechanicalSettings::RIGHT_COUNTS_PER_MM),
      wheelTrackMm(MechanicalSettings::WHEEL_TRACK_MM), cellSizeMm(180),
      maxLinearSpeedMmS(100), minLinearSpeedMmS(8), linearAccelerationMmS2(160),
      maxAngularSpeedDegS(55), minAngularSpeedDegS(10), angularAccelerationDegS2(240),
      turnKpDegPerSecPerDeg(3), positionToleranceMm(1), angleToleranceDeg(0.6f),
      settledSpeedMmS(8), settledGyroDegS(4), straightCorrectionKp(5),
      wallAlignmentKp(0.6f), wallAlignmentDeadbandMm(3.0f), leftFeedForwardPwmPerMmS(0.4f), rightFeedForwardPwmPerMmS(0.4f),
      leftStaticFrictionPwm(90), rightStaticFrictionPwm(90), imuYawSign(1),
      forwardTimeoutMs(6000), turnTimeoutMs(11000), stallTimeoutMs(700), stallMinSpeedMmS(5) {}

MotionController::MotionController(MotorDRV8833& m, EncoderN20& l, EncoderN20& r,
    IMUMPU6500& i, Odometry& o, PIDController& lp, PIDController& rp)
    : motors_(m), leftEncoder_(l), rightEncoder_(r), imu_(i), odometry_(o), leftPid_(lp), rightPid_(rp) {}

bool MotionController::begin(const MotionControllerConfig& c, const Pose2D& pose) {
    if (!isfinite(c.leftCountsPerMm) || !isfinite(c.rightCountsPerMm) ||
        c.leftCountsPerMm <= 0 || c.rightCountsPerMm <= 0 || c.wheelTrackMm <= 0 ||
        c.cellSizeMm <= 0 || c.maxLinearSpeedMmS <= 0 || c.maxLinearSpeedMmS > 100 ||
        c.linearAccelerationMmS2 <= 0 || c.maxAngularSpeedDegS <= 0 || c.maxAngularSpeedDegS > 55 ||
        c.minLinearSpeedMmS <= 0 || c.minAngularSpeedDegS <= 0 || c.turnKpDegPerSecPerDeg <= 0 ||
        c.positionToleranceMm <= 0 || c.angleToleranceDeg <= 0 || fabsf(c.imuYawSign) != 1 ||
        !isfinite(c.wallAlignmentKp) || c.wallAlignmentKp < 0 ||
        !isfinite(c.wallAlignmentDeadbandMm) || c.wallAlignmentDeadbandMm < 0 ||
        !c.forwardTimeoutMs || !c.turnTimeoutMs || !c.stallTimeoutMs ||
        !leftEncoder_.isInitialized() || !rightEncoder_.isInitialized() || !imu_.isInitialized()) return false;
    int32_t l = 0, r = 0;
    if (!leftEncoder_.readCount(l) || !rightEncoder_.readCount(r)) return false;
    OdometryConfig oc;
    oc.leftCountsPerMm = c.leftCountsPerMm; oc.rightCountsPerMm = c.rightCountsPerMm; oc.wheelTrackMm = c.wheelTrackMm;
    if (!odometry_.begin(oc, pose, l, r)) return false;
    MPU6500Reading initialReading{};
    if (!imu_.read(initialReading) || !isfinite(initialReading.yawDegrees)) return false;
    straightReferenceYawDeg_ = initialReading.yawDegrees * c.imuYawSign;
    frontApproachMode_ = false;
    config_ = c; initialized_ = true; status_ = MotionStatus::Idle; motionType_ = MotionType::None;
    failureReason_ = MotionFailureReason::None; braking_ = false; sensorPaused_ = false;
    startLeftCount_ = previousLeftCount_ = l;
    startRightCount_ = previousRightCount_ = r;
    return true;
}

bool MotionController::prepareMotion(MotionType type, uint32_t timeout) {
    if (status_ == MotionStatus::Running) return false;
    if (!initialized_) { fail(MotionFailureReason::NotInitialized); return false; }
    if (!motors_.isEnabled()) { fail(MotionFailureReason::MotorsDisabled); return false; }
    if (!leftEncoder_.readCount(startLeftCount_) || !rightEncoder_.readCount(startRightCount_)) {
        fail(MotionFailureReason::EncoderReadFailed); return false;
    }
    MPU6500Reading reading{};
    if (!imu_.read(reading) || !isfinite(reading.yawDegrees) || !isfinite(reading.gyroZdps)) { fail(MotionFailureReason::ImuReadFailed); return false; }
    // Preserve absolute yaw for PositionTracker. Each segment uses a relative delta.
    lastImuYawDeg_ = reading.yawDegrees;
    turnStartHeadingRad_ = odometry_.getPose().headingRad;
    previousLeftCount_ = startLeftCount_; previousRightCount_ = startRightCount_;
    motionStartedMs_ = lastLeftProgressMs_ = lastRightProgressMs_ = millis();
    sensorPaused_ = false;
    motionTimeoutMs_ = timeout; wallAlignmentValid_ = false; braking_ = false; brakeFailure_ = MotionFailureReason::None;
    guidedSteering_ = false;
    guidedRetreat_ = guidedPivot_ = pivotTrimUsed_ = false;
    arcPathWarning_=false;arcHardLimit_=0;
    distanceTravelledMm_ = turnAngleDeg_ = profiledLinearSpeedMmS_ = profiledAngularSpeedDegS_ = 0;
    leftIntegralPwm_ = rightIntegralPwm_ = 0;
    lastLeftMeasuredMmS_ = lastRightMeasuredMmS_ = 0;
    lastLeftTargetMmS_ = lastRightTargetMmS_ = 0;
    lastAngularProgress_ = 0; lastAngularProgressMs_ = motionStartedMs_;
    leftPid_.reset(); rightPid_.reset();
    motionType_ = type; failureReason_ = MotionFailureReason::None; status_ = MotionStatus::Running;
    return true;
}
bool MotionController::startMoveForwardOneCell() { return startMoveLinear(config_.cellSizeMm); }
bool MotionController::startMoveBackwardOneCell() { return startMoveLinear(-config_.cellSizeMm); }
bool MotionController::startMoveLinear(float mm) {
    if (status_ == MotionStatus::Running) return false;
    if (!isfinite(mm) || fabsf(mm) < 0.5f || fabsf(mm) > config_.cellSizeMm) {
        fail(MotionFailureReason::InvalidDistance); return false;
    }
    const uint32_t timeout=frontApproachMode_ && config_.forwardTimeoutMs<FrontApproach::TIMEOUT_MS
        ? FrontApproach::TIMEOUT_MS : config_.forwardTimeoutMs;
    if (!prepareMotion(MotionType::ForwardCell, timeout)) return false;
    targetDistanceMm_ = mm; return true;
}
bool MotionController::startTurn(float angle, MotionType type) {
    if (!prepareMotion(type, config_.turnTimeoutMs)) return false;
    targetAngleDeg_ = angle; return true;
}
bool MotionController::startTurnLeft90() { return startTurn(90, MotionType::TurnLeft90); }
bool MotionController::startGuidedPivot(float degrees) {
    const float magnitude = fabsf(degrees);
    if (!isfinite(degrees) || magnitude < 10.0f || magnitude > 90.0f ||
        !startTurn(degrees, degrees > 0.0f ? MotionType::TurnLeft90 : MotionType::TurnRight90)) return false;
    guidedPivot_ = true;
    guidedPivotHeadingStepDeg_ = degrees;
    targetAngleDeg_ = wrap(straightReferenceYawDeg_ + degrees - lastImuYawDeg_ * config_.imuYawSign);
    const float directedTarget = targetAngleDeg_ * sign(degrees);
    if (directedTarget < fmaxf(3.0f, magnitude - 20.0f) || directedTarget > magnitude + 20.0f) {
        fail(MotionFailureReason::SafetyLimit);
        return false;
    }
    return true;
}

bool MotionController::startRollingArc90(float degrees, float radiusMm) {
    // Mantém ambas as rodas em movimento; raio pequeno inclui ré na interna.
    if (!isfinite(degrees) || fabsf(fabsf(degrees)-90.0f)>0.01f ||
        !isfinite(radiusMm) || radiusMm<10.0f || radiusMm>40.0f) return false;
    if (!startTurn(degrees, degrees>0 ? MotionType::ArcLeft90 : MotionType::ArcRight90)) return false;
    arcRadiusMm_=radiusMm;arcStartPose_=odometry_.getPose();
    return true;
}

bool MotionController::startGuidedPivotReturn() {
    const float degrees = -turnAngleDeg_;
    if (!guidedPivot_ || status_ != MotionStatus::Failed || failureReason_ != MotionFailureReason::Obstacle ||
        fabsf(degrees) < 1.2f ||
        fabsf(degrees) > 75.0f || fabsf(distanceTravelledMm_) > guidedPivotMaxDriftMm ||
        !startTurn(degrees, degrees > 0.0f ? MotionType::TurnLeft90 : MotionType::TurnRight90)) return false;
    guidedPivot_ = true;
    guidedPivotHeadingStepDeg_ = 0.0f;
    pivotTrimUsed_ = true; // Restauração lenta, até 10 graus/s.
    return true;
}

bool MotionController::startGuidedRetreat(float distanceMm) {
    if (distanceMm <= 0.0f || !startMoveLinear(-distanceMm)) return false;
    guidedRetreat_ = true;
    retreatHeadingDeg_ = lastImuYawDeg_ * config_.imuYawSign;
    return true;
}
bool MotionController::startTurnRight90() { return startTurn(-90, MotionType::TurnRight90); }
bool MotionController::startTurnAround180() { return startTurn(-180, MotionType::TurnAround180); }
void MotionController::setFrontApproachMode(bool enabled) {
    if(status_!=MotionStatus::Running){frontApproachMode_=enabled;return;}
    if(enabled && isMovingForward() && !frontApproachMode_) {
        frontApproachMode_=true;
        profiledLinearSpeedMmS_=fminf(profiledLinearSpeedMmS_,FrontApproach::MAX_SPEED_MM_S);
        if(motionTimeoutMs_<FrontApproach::TIMEOUT_MS)motionTimeoutMs_=FrontApproach::TIMEOUT_MS;
        wallAlignmentValid_=false;
    }
}

bool MotionController::refineFrontApproachTarget(float frontCm, float previousProgressMm) {
    if(status_!=MotionStatus::Running || !isMovingForward() || !frontApproachMode_ ||
       braking_ || sensorPaused_ || !isfinite(frontCm) ||
       frontCm<=FrontApproach::TARGET_CM+FrontApproach::TOLERANCE_CM || frontCm>5.0f)return false;
    const float target=distanceTravelledMm_+(frontCm-FrontApproach::TARGET_CM)*10.0f;
    if(!FrontApproach::positionMatchesCell(previousProgressMm + distanceTravelledMm_,config_.cellSizeMm) ||
       !FrontApproach::positionMatchesCell(previousProgressMm + target,config_.cellSizeMm))return false;
    targetDistanceMm_=target;
    return true;
}

void MotionController::setWallAlignmentFeedback(float l, float r, bool valid) {
    // Only close, plausible corridor walls may steer; openings never pull
    // the chassis sideways. IMU still controls heading when this is unavailable.
    valid = valid && isfinite(l) && isfinite(r) && l >= 20 && r >= 20 &&
            l <= 80 && r <= 80 && l + r >= 40 && l + r <= 120;
    // Caller supplies a median from distinct fresh hardware frames.
    if (valid) { leftWallDistanceMm_ = l; rightWallDistanceMm_ = r; }
    wallAlignmentValid_ = valid;
}

void MotionController::setGuidedLateralSteering(bool slow, float yawOffsetDeg) {
    guidedSteering_ = true;
    guidedLateralSlow_ = slow;
    guidedYawOffsetDeg_ = isfinite(yawOffsetDeg) ? clamp(yawOffsetDeg, -6.0f, 6.0f) : 0.0f;
}

void MotionController::beginBrake(uint32_t now, MotionFailureReason reason) {
    if (braking_) return;
    motors_.brake(); braking_ = true; brakeStartedMs_ = quietSinceMs_ = now; brakeFailure_ = reason;
}
bool MotionController::sensorPauseSettled() const {
    return sensorPaused_ && millis()-sensorPausedAtMs_>=200 && millis()-quietSinceMs_>=100 &&
        fabsf(lastLeftMeasuredMmS_)<=config_.settledSpeedMmS &&
        fabsf(lastRightMeasuredMmS_)<=config_.settledSpeedMmS &&
        fabsf(currentGyroDps_)<config_.settledGyroDegS;
}
bool MotionController::setSensorPaused(bool paused) {
    if(status_!=MotionStatus::Running)return false;
    if(paused==sensorPaused_)return true;
    const uint32_t now=millis();
    if(paused) {
        sensorPaused_=true;sensorPausedAtMs_=now;
        motors_.brake();
    } else {
        if(!sensorPauseSettled())return false;
        const uint32_t duration=now-sensorPausedAtMs_;
        motionStartedMs_+=duration;
        if(braking_)brakeStartedMs_+=duration;
        lastLeftProgressMs_=lastRightProgressMs_=lastAngularProgressMs_=now;
        sensorPaused_=false;
    }
    profiledLinearSpeedMmS_=profiledAngularSpeedDegS_=0;
    lastLeftTargetMmS_=lastRightTargetMmS_=0;
    leftIntegralPwm_=rightIntegralPwm_=0;leftPid_.reset();rightPid_.reset();
    wallAlignmentValid_=false;
    return true;
}

void MotionController::update(float dt) {
    if (status_ != MotionStatus::Running) return;
    if (!isfinite(dt) || dt <= 0 || dt > 0.1f) { fail(MotionFailureReason::InvalidTimeStep); return; }
    int32_t l = 0, r = 0;
    if (!leftEncoder_.readCount(l) || !rightEncoder_.readCount(r)) { fail(MotionFailureReason::EncoderReadFailed); return; }
    MPU6500Reading reading{};
    if (!imu_.read(reading) || !isfinite(reading.yawDegrees) || !isfinite(reading.gyroZdps)) { fail(MotionFailureReason::ImuReadFailed); return; }
    const uint32_t now = millis();
    const int32_t dl = l - previousLeftCount_, dr = r - previousRightCount_;
    if (dl || dr) quietSinceMs_ = now;
    if (motionType_ == MotionType::ForwardCell) {
        const float direction = sign(targetDistanceMm_);
        updateStallTimers(dl * direction > 0.0f ? dl : 0, dr * direction > 0.0f ? dr : 0, now);
    } else {
        updateStallTimers(dl, dr, now);
    }
    const float alpha = dt / (0.035f + dt);
    lastLeftMeasuredMmS_ += alpha * (dl / config_.leftCountsPerMm / dt - lastLeftMeasuredMmS_);
    lastRightMeasuredMmS_ += alpha * (dr / config_.rightCountsPerMm / dt - lastRightMeasuredMmS_);
    previousLeftCount_ = l; previousRightCount_ = r;
    turnAngleDeg_ += wrap(reading.yawDegrees - lastImuYawDeg_) * config_.imuYawSign;
    lastImuYawDeg_ = reading.yawDegrees;
    currentGyroDps_ = reading.gyroZdps * config_.imuYawSign;
    if (!odometry_.update(l, r)) { fail(MotionFailureReason::EncoderReadFailed); return; }
    odometry_.setHeadingRad(turnStartHeadingRad_ + turnAngleDeg_ * pi / 180.0f);
    const auto travel = getWheelTravel();
    distanceTravelledMm_ = (travel.leftMm + travel.rightMm) * 0.5f;
    if(sensorPaused_) { motors_.brake();return; }
    if (braking_) {
        const bool settled = now - brakeStartedMs_ >= 300 && now - quietSinceMs_ >= 100 && fabsf(currentGyroDps_) < 4;
        if (settled) {
            if (guidedPivot_ && brakeFailure_ == MotionFailureReason::None) {
                const float error = fabsf(targetAngleDeg_ - turnAngleDeg_);
                if (fabsf(distanceTravelledMm_) > guidedPivotMaxDriftMm ||
                    (error > guidedPivotToleranceDeg && (pivotTrimUsed_ || error > 5.0f))) {
                    fail(MotionFailureReason::SafetyLimit);
                    return;
                }
                if (error > guidedPivotToleranceDeg) {
                    pivotTrimUsed_ = true;
                    braking_ = false;
                    lastLeftProgressMs_ = lastRightProgressMs_ = lastAngularProgressMs_ = now;
                    lastAngularProgress_ = turnAngleDeg_;
                    leftIntegralPwm_ = rightIntegralPwm_ = 0;
                    profiledAngularSpeedDegS_ = 0;
                    return;
                }
            }
            if (brakeFailure_ == MotionFailureReason::None) succeed(); else fail(brakeFailure_);
        } else if (now - brakeStartedMs_ >= 1000) fail(MotionFailureReason::BrakeNotSettled);
        return;
    }
    if (motionType_ == MotionType::ForwardCell) updateForward(dt, l, r, lastLeftMeasuredMmS_, lastRightMeasuredMmS_, now);
    else if (motionType_ == MotionType::ArcLeft90 || motionType_ == MotionType::ArcRight90)
        updateArc(dt, lastLeftMeasuredMmS_, lastRightMeasuredMmS_, now);
    else updateTurn(dt, lastLeftMeasuredMmS_, lastRightMeasuredMmS_, now);
}

void MotionController::updateForward(float dt, int32_t l, int32_t r, float ls, float rs, uint32_t now) {
    const float dir = sign(targetDistanceMm_);
    const float lt = (l - startLeftCount_) / config_.leftCountsPerMm * dir;
    const float rt = (r - startRightCount_) / config_.rightCountsPerMm * dir;
    const float target = fabsf(targetDistanceMm_);
    if (lt < -5 || rt < -5) {
        beginBrake(now, MotionFailureReason::WrongDirection); return;
    }
    if (lt > target + 20 || rt > target + 20 ||
        fabsf(rt - lt) > 20 || fabsf(turnAngleDeg_) > (guidedRetreat_ ? 5.0f : 15.0f)) {
        beginBrake(now, MotionFailureReason::SafetyLimit); return;
    }
    if (checkTimeoutAndStall(now, 1, 1)) return;
    const float remaining = target - (lt + rt) * 0.5f;
    const float lead = fmaxf(config_.positionToleranceMm, fmaxf(0, (ls + rs) * 0.5f * dir) * 0.07f);
    if (remaining <= lead) { beginBrake(now); return; }
    const bool guided = dir > 0.0f && guidedSteering_;
    const float segmentMaxSpeed = frontApproachMode_ ? FrontApproach::MAX_SPEED_MM_S : config_.maxLinearSpeedMmS;
    float maxSpeed = guided && guidedLateralSlow_ ? fminf(segmentMaxSpeed, 35.0f) : segmentMaxSpeed;
    if (guidedRetreat_) maxSpeed = fminf(maxSpeed, 18.0f);
    const float acceleration = frontApproachMode_ ? 40.0f : config_.linearAccelerationMmS2;
    const float deceleration = frontApproachMode_ ? 40.0f : 120.0f;
    const float desired = fminf(maxSpeed, fmaxf(config_.minLinearSpeedMmS, sqrtf(2 * deceleration * remaining)));
    const float wheelSpeedFloor = guided || guidedRetreat_ ? fminf(config_.minLinearSpeedMmS, desired) : 0.0f;
    profiledLinearSpeedMmS_ = fminf(desired, fmaxf(wheelSpeedFloor, profiledLinearSpeedMmS_ + acceleration * dt));
    // Keep a common heading reference over consecutive cells; resetting it
    // at every cell would retain any previous angular error.
    const float headingOffset = guided ? guidedYawOffsetDeg_ : 0.0f;
    const float reference = guidedRetreat_ ? retreatHeadingDeg_ : straightReferenceYawDeg_;
    const float headingError = wrap(lastImuYawDeg_ * config_.imuYawSign - reference - headingOffset);
    const float wallError = leftWallDistanceMm_ - rightWallDistanceMm_;
    const float outsideDeadband = sign(wallError) *
        fmaxf(0, fabsf(wallError) - config_.wallAlignmentDeadbandMm);
    const float wallCorrection = wallAlignmentValid_ && !frontApproachMode_ && !guided
        ? clamp(dir * config_.wallAlignmentKp * outsideDeadband, -6, 6) : 0;
    // Durante a rampa, preserve tração nas duas rodas ao corrigir o rumo.
    const float correctionLimit = guided || guidedRetreat_
        ? fminf(guidedRetreat_ ? 4.0f : (frontApproachMode_ ? 8.0f : 12.0f),
                profiledLinearSpeedMmS_ - wheelSpeedFloor)
        : 12.0f;
    const float correction = clamp(headingError * config_.straightCorrectionKp - wallCorrection,
                                   -correctionLimit, correctionLimit);
    const float leftTarget = dir * fmaxf(0, profiledLinearSpeedMmS_ + dir * correction);
    const float rightTarget = dir * fmaxf(0, profiledLinearSpeedMmS_ - dir * correction);
    applyWheelSpeedTargets(leftTarget, rightTarget, ls, rs, dt);
}

float MotionController::getArcPathErrorMm() const {
    return arcRadiusMm_*fmaxf(0.0f,turnAngleDeg_*sign(targetAngleDeg_))*pi/180.0f-distanceTravelledMm_;
}
float MotionController::getArcExitProgressMm() const {
    const auto pose=odometry_.getPose();
    const float exitHeading=turnStartHeadingRad_+sign(targetAngleDeg_)*pi*0.5f;
    return (pose.xMm-arcStartPose_.xMm)*cosf(exitHeading)+(pose.yMm-arcStartPose_.yMm)*sinf(exitHeading);
}

void MotionController::updateArc(float dt,float ls,float rs,uint32_t now) {
    const float dir=sign(targetAngleDeg_), angle=turnAngleDeg_*dir;
    const auto travel=getWheelTravel();
    const float expectedCentre=arcRadiusMm_*fmaxf(0.0f,angle)*pi/180.0f;
    const float leftSign=sign(arcRadiusMm_-dir*config_.wheelTrackMm*0.5f);
    const float rightSign=sign(arcRadiusMm_+dir*config_.wheelTrackMm*0.5f);
    if (travel.leftMm*leftSign < -4 || travel.rightMm*rightSign < -4) {
        beginBrake(now,MotionFailureReason::WrongDirection);return;
    }
    // Diferença para o arco ideal é diagnóstico e realimentação, não colisão.
    const float pathError=expectedCentre-distanceTravelledMm_;
    if(fabsf(pathError)>12.0f)arcPathWarning_=true;
    if(angle < -4 || angle>100){
        arcHardLimit_=1;beginBrake(now,MotionFailureReason::SafetyLimit);return;
    }
    if(fabsf(travel.leftMm)>180 || fabsf(travel.rightMm)>180){
        arcHardLimit_=2;beginBrake(now,MotionFailureReason::SafetyLimit);return;
    }
    if (angle>lastAngularProgress_+0.5f) {lastAngularProgress_=angle;lastAngularProgressMs_=now;}
    if (now-lastAngularProgressMs_>=700) {beginBrake(now,MotionFailureReason::MotionTimeout);return;}
    if (checkTimeoutAndStall(now,leftSign,rightSign)) return;
    const float remaining=90.0f-angle;
    if (remaining<=fmaxf(0.5f,fmaxf(0.0f,currentGyroDps_*dir)*0.035f)) {beginBrake(now);return;}
    const float maxRate=fminf(16.0f,config_.maxAngularSpeedDegS);
    const float rate=clamp(remaining*config_.turnKpDegPerSecPerDeg,6.0f,maxRate);
    profiledAngularSpeedDegS_=fminf(rate,fmaxf(6.0f,profiledAngularSpeedDegS_+60.0f*dt));
    const float omega=profiledAngularSpeedDegS_*pi/180.0f;
    // Mais avanço por grau, principalmente se o eixo estiver atrasado no arco.
    const float effectiveRadius=clamp(arcRadiusMm_+3.0f+0.6f*pathError,
                                     fmaxf(10.0f,arcRadiusMm_-5.0f),fminf(45.0f,arcRadiusMm_+10.0f));
    applyWheelSpeedTargets(omega*(effectiveRadius-dir*config_.wheelTrackMm*0.5f),
                          omega*(effectiveRadius+dir*config_.wheelTrackMm*0.5f),ls,rs,dt);
}

void MotionController::updateTurn(float dt, float ls, float rs, uint32_t now) {
    const float dir = sign(targetAngleDeg_);
    const float angle = turnAngleDeg_ * dir;
    const float target = fabsf(targetAngleDeg_);
    const float l = (previousLeftCount_ - startLeftCount_) / config_.leftCountsPerMm * -dir;
    const float r = (previousRightCount_ - startRightCount_) / config_.rightCountsPerMm * dir;
    const float arc = target * pi / 180 * config_.wheelTrackMm * 0.5f;
    if ((guidedPivot_ && fabsf(distanceTravelledMm_) > 10.0f) ||
        angle < -4 || angle > target + 30 || l < -20 / config_.leftCountsPerMm ||
        r < -20 / config_.rightCountsPerMm || fabsf(l) > arc * 1.5f + 40 / config_.leftCountsPerMm ||
        fabsf(r) > arc * 1.5f + 40 / config_.rightCountsPerMm) {
        beginBrake(now, MotionFailureReason::SafetyLimit); return;
    }
    if (angle > lastAngularProgress_ + 0.5f ||
        (guidedPivot_ && angle < lastAngularProgress_ - 0.5f)) {
        lastAngularProgress_ = angle;
        lastAngularProgressMs_ = now;
    }
    if (now - lastAngularProgressMs_ >= 700) { beginBrake(now, MotionFailureReason::MotionTimeout); return; }
    if (checkTimeoutAndStall(now, 1, 1)) return;
    const float remaining = target - angle;
    if (guidedPivot_) {
        const float travelDirection = dir * sign(remaining);
        const float lead = fmaxf(0.5f, fmaxf(0.0f, currentGyroDps_ * travelDirection) * 0.035f);
        if (fabsf(remaining) <= lead || (!pivotTrimUsed_ && remaining < 0.0f)) {
            beginBrake(now);
            return;
        }
        const float maxRate = fminf(config_.maxAngularSpeedDegS, pivotTrimUsed_ ? 10.0f : 25.0f);
        const float minRate = fminf(8.0f, maxRate);
        const float rate = clamp(fabsf(remaining) * config_.turnKpDegPerSecPerDeg, minRate, maxRate);
        profiledAngularSpeedDegS_ = fminf(rate, fmaxf(minRate, profiledAngularSpeedDegS_ + 90.0f * dt));
        const float base = profiledAngularSpeedDegS_ * pi / 180 * config_.wheelTrackMm * 0.5f;
        // Mesma distância em sentidos opostos mantém o ponto médio do eixo parado.
        const float correction = clamp((r - l) * sign(remaining) * 2.0f, -0.4f * base, 0.4f * base);
        // Ajude a roda que avança se ela atrasar nos primeiros milímetros.
        // A compensação sai gradualmente e não interfere na rampa do giro.
        const float forwardMm = dir > 0.0f ? r : l;
        const float reverseMm = dir > 0.0f ? l : r;
        const float assistPwm = !pivotTrimUsed_ && now - motionStartedMs_ < 500
            ? clamp((reverseMm - forwardMm - 0.5f) * 6.0f, 0.0f, 16.0f) *
                clamp((4.0f - forwardMm) * 0.5f, 0.0f, 1.0f)
            : 0.0f;
        applyWheelSpeedTargets(-travelDirection * (base + correction),
                               travelDirection * (base - correction), ls, rs, dt,
                               false, false, assistPwm);
        return;
    }
    const float lead = fmaxf(config_.angleToleranceDeg, fmaxf(0, currentGyroDps_ * dir) * 0.04f);
    if (remaining <= lead) { beginBrake(now); return; }
    const float rate = fminf(config_.maxAngularSpeedDegS, fmaxf(config_.minAngularSpeedDegS, remaining * config_.turnKpDegPerSecPerDeg));
    const float base = rate * pi / 180 * config_.wheelTrackMm * 0.5f;
    const float correction = clamp((r - l) * 2, -20, 20);
    const bool brakeLeft = r - l < -5, brakeRight = r - l > 5;
    applyWheelSpeedTargets(-dir * fmaxf(5, base + correction), dir * fmaxf(5, base - correction), ls, rs, dt,
                          brakeLeft, brakeRight);
}

void MotionController::applyWheelSpeedTargets(float lt, float rt, float lm, float rm, float dt, bool bl, bool br,
                                              float forwardAssistPwm) {
    lastLeftTargetMmS_ = lt; lastRightTargetMmS_ = rt;
    const float le = fabsf(lt) - lm * sign(lt), re = fabsf(rt) - rm * sign(rt);
    if (!bl) leftIntegralPwm_ = clamp(leftIntegralPwm_ + 5 * le * dt, -80, 100);
    if (!br) rightIntegralPwm_ = clamp(rightIntegralPwm_ + 5 * re * dt, -80, 100);
    const float leftAssist = lt > 0.0f ? forwardAssistPwm : 0.0f;
    const float rightAssist = rt > 0.0f ? forwardAssistPwm : 0.0f;
    const int lp = static_cast<int>(clamp(config_.leftStaticFrictionPwm +
        config_.leftFeedForwardPwmPerMmS * fabsf(lt) + 1.5f * le + leftIntegralPwm_ + leftAssist, 0, 220));
    const int rp = static_cast<int>(clamp(config_.rightStaticFrictionPwm +
        config_.rightFeedForwardPwmPerMmS * fabsf(rt) + 1.5f * re + rightIntegralPwm_ + rightAssist, 0, 220));
    motors_.setControlledSpeeds(static_cast<int16_t>(sign(lt) * lp), static_cast<int16_t>(sign(rt) * rp),
                               bl || fabsf(lt) < 0.01f || (!guidedPivot_ && !lp),
                               br || fabsf(rt) < 0.01f || (!guidedPivot_ && !rp));
}
void MotionController::updateStallTimers(int32_t l, int32_t r, uint32_t now) {
    if (l) lastLeftProgressMs_ = now; if (r) lastRightProgressMs_ = now;
}
bool MotionController::checkTimeoutAndStall(uint32_t now, float, float) {
    MotionFailureReason reason = MotionFailureReason::None;
    if (now - motionStartedMs_ >= motionTimeoutMs_) reason = MotionFailureReason::MotionTimeout;
    else if (now - lastLeftProgressMs_ >= config_.stallTimeoutMs) reason = MotionFailureReason::LeftWheelStalled;
    else if (now - lastRightProgressMs_ >= config_.stallTimeoutMs) reason = MotionFailureReason::RightWheelStalled;
    if (reason == MotionFailureReason::None) return false;
    beginBrake(now, reason); return true;
}
void MotionController::fail(MotionFailureReason reason) {
    sensorPaused_=false;
    motors_.setEnabled(false); status_ = MotionStatus::Failed; failureReason_ = reason; braking_ = false;
    lastLeftTargetMmS_ = lastRightTargetMmS_ = 0;
    leftPid_.reset(); rightPid_.reset();
}
void MotionController::brakeAtFrontTarget() {
    if (status_ == MotionStatus::Running && frontApproachMode_ && isMovingForward()) beginBrake(millis());
}
void MotionController::brakeForObstacle() {
    if (status_ != MotionStatus::Running) return;
    sensorPaused_ = false;
    if (!braking_) beginBrake(millis(), MotionFailureReason::Obstacle);
    else if (brakeFailure_ == MotionFailureReason::None) brakeFailure_ = MotionFailureReason::Obstacle;
}
void MotionController::succeed() {
    sensorPaused_=false;
    if (guidedPivot_)
        straightReferenceYawDeg_ = wrap(straightReferenceYawDeg_ + guidedPivotHeadingStepDeg_);
    else if (motionType_ != MotionType::ForwardCell)
        straightReferenceYawDeg_ = lastImuYawDeg_ * config_.imuYawSign;
    motors_.setEnabled(false); status_ = MotionStatus::Succeeded; failureReason_ = MotionFailureReason::None; braking_ = false;
    lastLeftTargetMmS_ = lastRightTargetMmS_ = 0;
    leftPid_.reset(); rightPid_.reset();
}
void MotionController::stop() { fail(MotionFailureReason::Cancelled); }
MotionStatus MotionController::getStatus() const { return status_; }
MotionType MotionController::getMotionType() const { return motionType_; }
MotionFailureReason MotionController::getFailureReason() const { return failureReason_; }
Pose2D MotionController::getPose() const { return odometry_.getPose(); }
WheelTravel MotionController::getWheelTravel() const {
    const int32_t left = previousLeftCount_ - startLeftCount_;
    const int32_t right = previousRightCount_ - startRightCount_;
    return {left, right, left / config_.leftCountsPerMm, right / config_.rightCountsPerMm};
}
float MotionController::getDistanceTravelledMm() const { return distanceTravelledMm_; }
float MotionController::getTurnAngleDeg() const { return turnAngleDeg_; }
float MotionController::getLeftSpeedMmS() const { return lastLeftMeasuredMmS_; }
float MotionController::getRightSpeedMmS() const { return lastRightMeasuredMmS_; }
float MotionController::getPidErrorMmS() const { return ((lastLeftTargetMmS_ - lastLeftMeasuredMmS_) + (lastRightTargetMmS_ - lastRightMeasuredMmS_)) * 0.5f; }
