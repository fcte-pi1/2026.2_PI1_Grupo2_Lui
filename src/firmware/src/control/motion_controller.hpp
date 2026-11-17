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
    InvalidDistance,
    SafetyLimit,
    BrakeNotSettled
};

// Leituras das paredes vindas do subsistema de ultrassom, em milímetros.
// xValid=false quando a leitura está ausente ou é implausível; sem eco
// (caminho livre) também deve ser enviado como inválido.
struct WallFeedback {
    float frontMm = 0.0f;
    float leftMm = 0.0f;
    float rightMm = 0.0f;
    bool frontValid = false;
    bool leftValid = false;
    bool rightValid = false;
    uint32_t sampledAtMs = 0;
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

    // Ganho de correção de rumo por yaw da IMU; fatores de encoder são provisórios.
    // Unidade: (mm/s de correcao diferencial) / grau de yaw.
    float straightCorrectionKp;
    // Centralização lateral (cascata): o erro lateral em mm gera um alvo de
    // rumo em graus, seguido pela malha de rumo da IMU. Unidade: grau / mm.
    // Zero desativa a centralização.
    float wallAlignmentKp;
    // Limite do alvo de rumo gerado pelas paredes, em graus.
    float maxWallHeadingDeg;
    // Leitura lateral abaixo disto é considerada parede adjacente, em mm.
    float lateralWallMaxDistanceMm;
    // Distância sensor-parede com o robô centralizado. Zero = aprender em
    // movimento quando houver as duas paredes (ou na largada, pelo main).
    float lateralCenterDistanceMm;
    // Amostras de parede mais antigas que isto são ignoradas, em ms.
    uint32_t wallFeedbackMaxAgeMs;

    // Distância do sensor frontal à parede com o robô no centro da célula.
    // Zero desativa a parada pela parede frontal. Calibrar no labirinto.
    float frontStopDistanceMm;
    // Diferença máxima aceita entre o restante pela parede frontal e pelo
    // encoder; acima disto a parede vista não é a do fim desta célula.
    float frontMaxCorrectionMm;

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
    void setWallFeedback(const WallFeedback& feedback);
    // Distância lateral com o robô centralizado (ex.: medida na largada,
    // entre as duas paredes do beco). Valores <= 0 são ignorados.
    void setLateralCenterDistanceMm(float distanceMm);
    bool startTurnLeft90();
    bool startTurnRight90();
    bool startTurnAround180(); // meia-volta à direita, como na bancada

    // Chamar periodicamente pela Task_PID_Control (por exemplo, a cada 5 ms).
    void update(float dtSeconds);

    // Interrompe o movimento. Status passa a Failed com motivo Cancelled.
    void stop();

    bool isMovingForward() const { return motionType_ == MotionType::ForwardCell && targetDistanceMm_ > 0; }
    MotionStatus getStatus() const;
    MotionType getMotionType() const;
    MotionFailureReason getFailureReason() const;
    Pose2D getPose() const;
    float getDistanceTravelledMm() const;
    float getTurnAngleDeg() const;
    float getLeftSpeedMmS() const;
    float getRightSpeedMmS() const;
    float getPidErrorMmS() const;
    float getWallHeadingTargetDeg() const { return wallHeadingTargetDeg_; }
    float getLateralCenterDistanceMm() const { return lateralCenterMm_; }
    // Última correção aplicada pela parede frontal (restante frontal menos
    // restante do encoder), em mm. Zero quando não usada no movimento.
    float getFrontCorrectionMm() const { return frontCorrectionMm_; }

private:
    bool prepareMotion(MotionType type, uint32_t timeoutMs);
    bool startTurn(float targetAngleDeg, MotionType type);
    void updateForward(float dtSeconds, int32_t leftCount,
                       int32_t rightCount, float leftSpeedMmS,
                       float rightSpeedMmS, uint32_t nowMs);
    bool wallFeedbackFresh(uint32_t nowMs) const;
    void updateWallHeadingTarget(uint32_t nowMs);
    bool frontRemainingMm(uint32_t nowMs, float speedMmS,
                          float& remainingMm) const;
    void updateTurn(float dtSeconds, float leftSpeedMmS,
                    float rightSpeedMmS, uint32_t nowMs);
    void applyWheelSpeedTargets(float leftTargetMmS,
                                float rightTargetMmS,
                                float leftMeasuredMmS,
                                float rightMeasuredMmS,
                                float dtSeconds, bool brakeLeft = false, bool brakeRight = false);
    void beginBrake(uint32_t now, MotionFailureReason reason = MotionFailureReason::None);
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
    bool braking_ = false;
    uint32_t brakeStartedMs_ = 0, quietSinceMs_ = 0;
    uint32_t lastAngularProgressMs_ = 0;
    float lastAngularProgress_ = 0, currentGyroDps_ = 0;
    float leftIntegralPwm_ = 0, rightIntegralPwm_ = 0;
    MotionFailureReason brakeFailure_ = MotionFailureReason::None;

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
    WallFeedback wall_{};
    uint32_t lastWallSampleMs_ = 0;
    bool hasWallSample_ = false;
    float lateralCenterMm_ = 0.0f;
    float wallHeadingTargetDeg_ = 0.0f;
    float frontCorrectionMm_ = 0.0f;
    float distanceTravelledMm_ = 0.0f;
    float targetAngleDeg_ = 0.0f;
    float turnAngleDeg_ = 0.0f;
    float lastImuYawDeg_ = 0.0f;
    float turnStartHeadingRad_ = 0.0f;
    float profiledLinearSpeedMmS_ = 0.0f;
    float profiledAngularSpeedDegS_ = 0.0f;
    float lastLeftTargetMmS_ = 0.0f;
    float lastRightTargetMmS_ = 0.0f;
    float lastLeftMeasuredMmS_ = 0.0f;
    float lastRightMeasuredMmS_ = 0.0f;
};
