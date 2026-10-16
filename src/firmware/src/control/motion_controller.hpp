#pragma once

#include <stdint.h>

#include "pid_controller.hpp"
#include "odometry.hpp"
#include "../hal/motor_drv8833.hpp"
#include "../hal/encoder_n20.hpp"
#include "../hal/imu_mpu6500.hpp"

enum class MotionStatus : uint8_t {
    Idle,
    Running,
    Succeeded,
    Failed
};

enum class MotionType : uint8_t {
    None,
    ForwardCell,
    TurnLeft90,
    TurnRight90,
    TurnAround180
};

enum class MotionFailureReason : uint8_t {
    None,
    NotInitialized,
    MotorsDisabled,
    EncoderReadFailed,
    ImuReadFailed,
    InvalidTimeStep,
    MotionTimeout,
    LeftWheelStalled,
    RightWheelStalled,
    Cancelled,
    InvalidDistance
};

struct MotionControllerConfig {
    float leftCountsPerMm;
    float rightCountsPerMm;
    float wheelTrackMm;
    float cellSizeMm;

    float maxLinearSpeedMmS;
    float minLinearSpeedMmS;
    float linearAccelerationMmS2;

    float maxAngularSpeedDegS;
    float minAngularSpeedDegS;
    float angularAccelerationDegS2;
    float turnKpDegPerSecPerDeg;

    float positionToleranceMm;
    float angleToleranceDeg;
    float settledSpeedMmS;
    float settledGyroDegS;

    // Corrige a diferença entre distâncias das rodas durante avanço.
    // Unidade aproximada: (mm/s de correção) / mm de diferença.
    float straightCorrectionKp;
    // Correção diferencial a partir da diferença de distância dos sensores laterais.
    float wallAlignmentKp;

    // Feed-forward aproximado do motor: PWM por mm/s + compensação de atrito
    // estático. Calibrar para cada roda.
    float leftFeedForwardPwmPerMmS;
    float rightFeedForwardPwmPerMmS;
    float leftStaticFrictionPwm;
    float rightStaticFrictionPwm;

    // Sinal a aplicar ao yaw da IMU: +1 ou -1 conforme a montagem.
    float imuYawSign;

    uint32_t forwardTimeoutMs;
    uint32_t turnTimeoutMs;
    uint32_t stallTimeoutMs;
    float stallMinSpeedMmS;

    MotionControllerConfig();
};

class MotionController {
public:
    MotionController(MotorDRV8833& motors,
                     EncoderN20& leftEncoder,
                     EncoderN20& rightEncoder,
                     IMUMPU6500& imu,
                     Odometry& odometry,
                     PIDController& leftPid,
                     PIDController& rightPid);

    // Chamar após inicializar os drivers e zerar/calibrar sensores.
    // initialPose pode ser (0,0,0) se a referência local começar na largada.
    bool begin(const MotionControllerConfig& config,
               const Pose2D& initialPose);

    // Cada start... inicia uma operação e retorna sem bloquear.
    bool startMoveForwardOneCell();
    bool startMoveBackwardOneCell();
    // Distância assinada em milímetros: positiva avança, negativa dá ré.
    bool startMoveLinear(float signedDistanceMm);

    // Feedback atualizado pelo subsistema de sensores, em milímetros.
    // Leituras inválidas/desatualizadas devem ser enviadas com valid=false.
    void setWallAlignmentFeedback(float leftDistanceMm,
                                  float rightDistanceMm,
                                  bool valid);
    bool startTurnLeft90();
    bool startTurnRight90();
    bool startTurnAround180(); // por convenção, gira para a esquerda (+180 graus)

    // Chamar periodicamente pela Task_PID_Control (por exemplo, a cada 5 ms).
    void update(float dtSeconds);

    // Interrompe o movimento. Status passa a Failed com motivo Cancelled.
    void stop();

    MotionStatus getStatus() const;
    MotionType getMotionType() const;
    MotionFailureReason getFailureReason() const;
    Pose2D getPose() const;
    float getDistanceTravelledMm() const;
    float getTurnAngleDeg() const;

private:
    bool prepareMotion(MotionType type, uint32_t timeoutMs);
    bool startTurn(float targetAngleDeg, MotionType type);
    void updateForward(float dtSeconds, int32_t leftCount,
                       int32_t rightCount, float leftSpeedMmS,
                       float rightSpeedMmS, uint32_t nowMs);
    void updateTurn(float dtSeconds, float leftSpeedMmS,
                    float rightSpeedMmS, uint32_t nowMs);
    void applyWheelSpeedTargets(float leftTargetMmS,
                                float rightTargetMmS,
                                float leftMeasuredMmS,
                                float rightMeasuredMmS,
                                float dtSeconds);
    void fail(MotionFailureReason reason);
    void succeed();
    void updateStallTimers(int32_t leftDeltaCount,
                           int32_t rightDeltaCount,
                           uint32_t nowMs);
    bool checkTimeoutAndStall(uint32_t nowMs,
                              float leftTargetMmS,
                              float rightTargetMmS);

    MotorDRV8833& motors_;
    EncoderN20& leftEncoder_;
    EncoderN20& rightEncoder_;
    IMUMPU6500& imu_;
    Odometry& odometry_;
    PIDController& leftPid_;
    PIDController& rightPid_;

    MotionControllerConfig config_{};
    MotionStatus status_ = MotionStatus::Idle;
    MotionType motionType_ = MotionType::None;
    MotionFailureReason failureReason_ = MotionFailureReason::None;
    bool initialized_ = false;

    int32_t startLeftCount_ = 0;
    int32_t startRightCount_ = 0;
    int32_t previousLeftCount_ = 0;
    int32_t previousRightCount_ = 0;
    uint32_t motionStartedMs_ = 0;
    uint32_t motionTimeoutMs_ = 0;
    uint32_t lastLeftProgressMs_ = 0;
    uint32_t lastRightProgressMs_ = 0;
    uint8_t settledSamples_ = 0;

    float targetDistanceMm_ = 0.0f;
    float leftWallDistanceMm_ = 0.0f;
    float rightWallDistanceMm_ = 0.0f;
    bool wallAlignmentValid_ = false;
    float distanceTravelledMm_ = 0.0f;
    float targetAngleDeg_ = 0.0f;
    float turnAngleDeg_ = 0.0f;
    float lastImuYawDeg_ = 0.0f;
    float turnStartHeadingRad_ = 0.0f;
    float profiledLinearSpeedMmS_ = 0.0f;
    float profiledAngularSpeedDegS_ = 0.0f;
    float lastLeftTargetMmS_ = 0.0f;
    float lastRightTargetMmS_ = 0.0f;
};
