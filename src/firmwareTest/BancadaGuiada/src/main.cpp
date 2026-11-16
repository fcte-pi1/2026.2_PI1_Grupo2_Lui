#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config/board_pins.hpp"
#include "config/mechanical_config.hpp"
#include "bench_motor_outputs.hpp"
#include "continuous_yaw.hpp"
#include "hal/encoder_n20.hpp"
#include "hal/imu_mpu6500.hpp"
#include "hal/ultrasonic_us100.hpp"

BenchMotorOutputs motors;
EncoderN20 leftEncoder, rightEncoder;
IMUMPU6500 imu;
ContinuousTurnYaw continuousTurnYaw;
UltrasonicUS100 frontSensor, leftSensor, rightSensor;
bool stationaryActive = false;
uint32_t stationaryStartMs = 0, stationaryLastMs = 0, stationaryLogMs = 0;
float stationaryGyroSum = 0.0f, stationaryPeakGyro = 0.0f;
uint32_t stationarySamples = 0;
int32_t stationaryStartLeft = 0, stationaryStartRight = 0;
bool motorsReady = false;
bool encodersReady = false;
bool imuReady = false;
bool armed = false;
bool floorArmed = false;
bool turnActive = false;
bool turnBraking = false;
const char* turnReason = "";
uint32_t turnStartedMs = 0;
uint32_t turnLastUpdateMs = 0;
uint32_t turnLastProgressMs = 0;
uint32_t turnBrakingMs = 0;
uint32_t turnQuietSinceMs = 0;
int32_t turnStartLeft = 0, turnStartRight = 0;
int32_t turnLastLeft = 0, turnLastRight = 0;
float turnProgressDeg = 0.0f;
float turnAngleDeg = 0.0f;
float turnTargetDegrees = 30.0f;
float turnMaximumRateDps = 35.0f;
int turnDirectionSign = 1; // +1 direita, -1 esquerda
uint32_t turnTimeoutMs = 3000;
int32_t turnControlLastLeft = 0, turnControlLastRight = 0;
float turnLeftSpeedMmS = 0.0f, turnRightSpeedMmS = 0.0f;
float turnLeftIntegralPwm = 0.0f, turnRightIntegralPwm = 0.0f;
uint32_t turnLastLogMs = 0;
bool pulseActive = false;
bool wireReady = false;
uint32_t pulseStartedMs = 0;
uint32_t armedAtMs = 0;
bool rotationActive = false;
bool rotationBraking = false;
bool rotationLeft = true;
int32_t rotationStartCount = 0;
int32_t rotationPreviousDelta = 0;
uint32_t rotationStartedMs = 0;
uint32_t rotationProgressMs = 0;
uint32_t rotationBrakingMs = 0;
uint32_t rotationQuietSinceMs = 0;
int32_t rotationBrakeLastCount = 0;
struct ForwardTrial {
    bool active = false, braking = false;
    const char* reason = "";
    int32_t startLeft = 0, startRight = 0;
    int32_t previousLeft = 0, previousRight = 0;
    int32_t brakeLeft = 0, brakeRight = 0;
    uint32_t startedMs = 0, lastUpdateMs = 0, lastFrontMs = 0, lastLogMs = 0;
    uint32_t leftProgressMs = 0, rightProgressMs = 0, brakingMs = 0, quietSinceMs = 0;
    float leftSpeed = 0.0f, rightSpeed = 0.0f;
    float leftIntegral = 0.0f, rightIntegral = 0.0f;
    float yawDegrees = 0.0f;
    float targetMm = 30.0f;
    float maximumSpeedMmS = 30.0f;
    float commandedSpeedMmS = 0.0f;
    uint32_t timeoutMs = 4000;
};
ForwardTrial forwardTrial;


constexpr int32_t ROTATION_TARGET_COUNTS = 290; // estimativa a validar pela marca


void stopAll() {
    stationaryActive = false;
    motors.setEnabled(false);
    armed = false;
    floorArmed = false;
    turnActive = false;
    turnBraking = false;
    forwardTrial.active = false;
    forwardTrial.braking = false;
    pulseActive = false;
    rotationActive = false;
    rotationBraking = false;
}

void printCounts() {
    int32_t left = 0, right = 0;
    if (!encodersReady || !leftEncoder.readCount(left) || !rightEncoder.readCount(right)) {
        Serial.println("ENCODER: indisponivel");
        return;
    }
    Serial.printf("ENCODER L=%ld R=%ld\n", static_cast<long>(left), static_cast<long>(right));
}

void beginTurnBrake(const char* reason, uint32_t now, int32_t left, int32_t right) {
    motors.brake();
    turnBraking = true;
    turnReason = reason;
    turnBrakingMs = now;
    turnQuietSinceMs = now;
    turnLastLeft = left;
    turnLastRight = right;
    Serial.printf("GIRO: freando (%s), angulo=%.2f graus.\n", reason, turnAngleDeg);
}

void updateGuidedTurn(uint32_t now) {
    if (!turnActive || static_cast<uint32_t>(now - turnLastUpdateMs) < 5) return;
    const float dt = static_cast<float>(now - turnLastUpdateMs) / 1000.0f;
    turnLastUpdateMs = now;
    int32_t left = 0, right = 0;
    if (!leftEncoder.readCount(left) || !rightEncoder.readCount(right)) {
        stopAll();
        Serial.println("GIRO FALHOU: encoder indisponivel.");
        return;
    }
    MPU6500Reading reading{};
    const bool validImu = imu.read(reading);
    if (validImu) turnAngleDeg = continuousTurnYaw.update(reading.yawDegrees, turnDirectionSign);
    if (turnBraking) {
        if (left != turnLastLeft || right != turnLastRight) {
            turnLastLeft = left;
            turnLastRight = right;
            turnQuietSinceMs = now;
        }
        const bool settled = static_cast<uint32_t>(now - turnBrakingMs) >= 300 &&
                             static_cast<uint32_t>(now - turnQuietSinceMs) >= 100 &&
                             (!validImu || fabsf(reading.gyroZdps) < 4.0f);
        if (settled || static_cast<uint32_t>(now - turnBrakingMs) >= 1000) {
            const float angle = turnAngleDeg;
            const char* reason = turnReason;
            stopAll();
            Serial.printf("GIRO ENCERRADO: motivo=%s angulo_final=%.2f tempo=%.3fs repouso=%s IMU=%s; driver desabilitado.\n",
                          reason, angle, (now - turnStartedMs) / 1000.0f, settled ? "SIM" : "NAO", validImu ? "OK" : "FALHA");
            printCounts();
        }
        return;
    }
    if (!validImu || dt > 0.1f) {
        beginTurnBrake("falha IMU/periodo", now, left, right);
        return;
    }
    const int32_t dl = left - turnStartLeft, dr = right - turnStartRight;
    const float rate = -reading.gyroZdps * turnDirectionSign;
    if (turnAngleDeg > turnProgressDeg + 0.5f) {
        turnProgressDeg = turnAngleDeg;
        turnLastProgressMs = now;
    }
    const float expectedWheelArcMm = turnTargetDegrees * MechanicalSettings::kPi / 180.0f *
                                     MechanicalSettings::WHEEL_TRACK_MM * 0.5f;
    const int32_t maxLeftCounts = static_cast<int32_t>(ceilf(expectedWheelArcMm *
                                  MechanicalSettings::LEFT_COUNTS_PER_MM * 1.5f)) + 40;
    const int32_t maxRightCounts = static_cast<int32_t>(ceilf(expectedWheelArcMm *
                                   MechanicalSettings::RIGHT_COUNTS_PER_MM * 1.5f)) + 40;
    if (turnAngleDeg < -4.0f || turnAngleDeg > turnTargetDegrees + 30.0f || dl * turnDirectionSign < -20 || dr * turnDirectionSign > 20 ||
        labs(dl) > maxLeftCounts || labs(dr) > maxRightCounts) {
        beginTurnBrake("sinal/limite de movimento", now, left, right);
        return;
    }
    if (static_cast<uint32_t>(now - turnStartedMs) >= turnTimeoutMs ||
        static_cast<uint32_t>(now - turnLastProgressMs) >= 700 ||
        (static_cast<uint32_t>(now - turnStartedMs) > 500 && (labs(dl) < 3 || labs(dr) < 3))) {
        beginTurnBrake("tempo/sem progresso", now, left, right);
        return;
    }
    const float remaining = turnTargetDegrees - turnAngleDeg;
    // Bench trials settled only ~0.2-0.4 deg after braking near target.
    // Keep speed-dependent anticipation, reducing the fixed 2-deg early stop.
    const float brakingLead = fmaxf(0.6f, fmaxf(rate, 0.0f) * 0.04f);
    if (remaining <= brakingLead) {
        beginTurnBrake("aproximacao do alvo angular", now, left, right);
        return;
    }
    // Velocidade e progresso controlados por roda, com sinais de giro normalizados.
    // Os sinais se invertem no giro a esquerda; ambas percorrem o mesmo arco.
    const float leftProgress = dl * turnDirectionSign / MechanicalSettings::LEFT_COUNTS_PER_MM;
    const float rightProgress = -dr * turnDirectionSign / MechanicalSettings::RIGHT_COUNTS_PER_MM;
    const float leftSpeed = (left - turnControlLastLeft) * turnDirectionSign / MechanicalSettings::LEFT_COUNTS_PER_MM / dt;
    const float rightSpeed = -(right - turnControlLastRight) * turnDirectionSign / MechanicalSettings::RIGHT_COUNTS_PER_MM / dt;
    turnControlLastLeft = left;
    turnControlLastRight = right;
    const float alpha = dt / (0.035f + dt);
    turnLeftSpeedMmS += alpha * (leftSpeed - turnLeftSpeedMmS);
    turnRightSpeedMmS += alpha * (rightSpeed - turnRightSpeedMmS);
    const float approachGain = turnMaximumRateDps >= 55.0f ? 3.0f : 2.0f;
    const float desiredRate = fminf(turnMaximumRateDps, fmaxf(10.0f, remaining * approachGain));
    const float baseSpeed = desiredRate * MechanicalSettings::kPi / 180.0f *
                            MechanicalSettings::WHEEL_TRACK_MM * 0.5f;
    const float progressError = rightProgress - leftProgress;
    const float correction = constrain(progressError * 2.0f, -20.0f, 20.0f);
    const float leftTarget = fmaxf(5.0f, baseSpeed + correction);
    const float rightTarget = fmaxf(5.0f, baseSpeed - correction);
    // Se uma roda ganha mais de 5 mm, segura-a enquanto a outra recupera.
    bool brakeLeft = progressError < -5.0f;
    bool brakeRight = progressError > 5.0f;
    const float leftError = leftTarget - turnLeftSpeedMmS;
    const float rightError = rightTarget - turnRightSpeedMmS;
    if (!brakeLeft) turnLeftIntegralPwm = constrain(turnLeftIntegralPwm + 5.0f * leftError * dt, -80.0f, 100.0f);
    if (!brakeRight) turnRightIntegralPwm = constrain(turnRightIntegralPwm + 5.0f * rightError * dt, -80.0f, 100.0f);
    int leftPwm = static_cast<int>(constrain(90.0f + 0.4f * leftTarget + 1.5f * leftError + turnLeftIntegralPwm, 0.0f, 220.0f));
    int rightPwm = static_cast<int>(constrain(90.0f + 0.4f * rightTarget + 1.5f * rightError + turnRightIntegralPwm, 0.0f, 220.0f));
    brakeLeft = brakeLeft || leftPwm == 0;
    brakeRight = brakeRight || rightPwm == 0;
    motors.setTurnOutputs(turnDirectionSign * leftPwm, -turnDirectionSign * rightPwm, brakeLeft, brakeRight);
    if (static_cast<uint32_t>(now - turnLastLogMs) >= 100) {
        turnLastLogMs = now;
        Serial.printf("GIRO RODAS: ang=%.2f L=%.1fmm R=%.1fmm pwm=%d/%d freio=%u/%u\n",
                      turnAngleDeg, leftProgress, rightProgress, leftPwm, rightPwm,
                      static_cast<unsigned>(brakeLeft), static_cast<unsigned>(brakeRight));
    }
}

void beginForwardBrake(const char* reason, uint32_t now, int32_t left, int32_t right) {
    motors.brake();
    forwardTrial.braking = true;
    forwardTrial.reason = reason;
    forwardTrial.brakingMs = now;
    forwardTrial.quietSinceMs = now;
    forwardTrial.brakeLeft = left;
    forwardTrial.brakeRight = right;
    Serial.printf("AVANCO: freando (%s).\n", reason);
}

void updateGuidedForward(uint32_t now) {
    auto& f = forwardTrial;
    if (!f.active || static_cast<uint32_t>(now - f.lastUpdateMs) < 5) return;
    const float dt = static_cast<float>(now - f.lastUpdateMs) / 1000.0f;
    f.lastUpdateMs = now;
    int32_t left = 0, right = 0;
    if (!leftEncoder.readCount(left) || !rightEncoder.readCount(right)) {
        stopAll();
        Serial.println("AVANCO FALHOU: encoder indisponivel.");
        return;
    }
    MPU6500Reading reading{};
    const bool imuValid = imu.read(reading);
    if (imuValid) f.yawDegrees = reading.yawDegrees;
    const float leftMm = (left - f.startLeft) / MechanicalSettings::LEFT_COUNTS_PER_MM;
    const float rightMm = (right - f.startRight) / MechanicalSettings::RIGHT_COUNTS_PER_MM;
    if (f.braking) {
        if (left != f.brakeLeft || right != f.brakeRight) {
            f.brakeLeft = left;
            f.brakeRight = right;
            f.quietSinceMs = now;
        }
        const bool settled = static_cast<uint32_t>(now - f.brakingMs) >= 300 &&
                             static_cast<uint32_t>(now - f.quietSinceMs) >= 100 &&
                             (!imuValid || fabsf(reading.gyroZdps) < 4.0f);
        if (settled || static_cast<uint32_t>(now - f.brakingMs) >= 1000) {
            const char* reason = f.reason;
            const float yaw = f.yawDegrees;
            stopAll();
            Serial.printf("AVANCO ENCERRADO: motivo=%s L=%.2fmm R=%.2fmm media=%.2fmm yaw=%.2f tempo=%.3fs repouso=%s; driver desabilitado.\n",
                          reason, leftMm, rightMm, (leftMm + rightMm) * 0.5f, yaw, (now - f.startedMs) / 1000.0f,
                          settled ? "SIM" : "NAO");
            printCounts();
        }
        return;
    }
    if (!imuValid || dt > 0.1f || fabsf(f.yawDegrees) > 15.0f ||
        leftMm < -5.0f || rightMm < -5.0f || leftMm > f.targetMm + 20.0f || rightMm > f.targetMm + 20.0f) {
        beginForwardBrake("IMU/periodo/sinal/limite de deslocamento", now, left, right);
        return;
    }
    if (left != f.previousLeft) f.leftProgressMs = now;
    if (right != f.previousRight) f.rightProgressMs = now;
    if (static_cast<uint32_t>(now - f.startedMs) >= f.timeoutMs ||
        static_cast<uint32_t>(now - f.leftProgressMs) >= 700 ||
        static_cast<uint32_t>(now - f.rightProgressMs) >= 700) {
        beginForwardBrake("tempo/roda sem progresso", now, left, right);
        return;
    }
    if (static_cast<uint32_t>(now - f.lastFrontMs) >= 70) {
        f.lastFrontMs = now;
        const UltrasonicReading front = frontSensor.measure();
        if (!front.valid || front.distanceCm <= 5.0f) {
            beginForwardBrake(front.valid ? "obstaculo frontal <=5cm" : "sensor frontal invalido", millis(), left, right);
            return;
        }
    }
    const float leftSpeed = (left - f.previousLeft) / MechanicalSettings::LEFT_COUNTS_PER_MM / dt;
    const float rightSpeed = (right - f.previousRight) / MechanicalSettings::RIGHT_COUNTS_PER_MM / dt;
    f.previousLeft = left;
    f.previousRight = right;
    const float alpha = dt / (0.035f + dt);
    f.leftSpeed += alpha * (leftSpeed - f.leftSpeed);
    f.rightSpeed += alpha * (rightSpeed - f.rightSpeed);
    const float remaining = f.targetMm - (leftMm + rightMm) * 0.5f;
    const float lead = fmaxf(1.0f, fmaxf(0.0f, (f.leftSpeed + f.rightSpeed) * 0.5f) * 0.07f);
    if (remaining <= lead) {
        beginForwardBrake("aproximacao do alvo", now, left, right);
        return;
    }
    float baseSpeed = fminf(30.0f, fmaxf(8.0f, remaining * 1.5f));
    if (f.maximumSpeedMmS > 30.0f) {
        // Tune only the 100 mm/s profile; keep 60/80 mm/s references unchanged.
        const float acceleration = f.maximumSpeedMmS >= 100.0f ? 160.0f : 120.0f;
        const float deceleration = f.maximumSpeedMmS >= 100.0f ? 120.0f : 80.0f;
        const float approachSpeed = sqrtf(2.0f * deceleration * fmaxf(remaining, 0.0f));
        const float desiredSpeed = fminf(f.maximumSpeedMmS, fmaxf(8.0f, approachSpeed));
        f.commandedSpeedMmS = fminf(desiredSpeed, f.commandedSpeedMmS + acceleration * dt);
        baseSpeed = f.commandedSpeedMmS;
    }
    const float progressError = rightMm - leftMm;
    // Encoder scale factors are provisional: prefer IMU heading over exact
    // equality of accumulated wheel distances, retaining a bounded synchronizer.
    const bool fastProfile = f.maximumSpeedMmS > 30.0f;
    // Fast profile: inner loops still use encoder velocity, but IMU alone
    // sets heading correction. Provisional wheel scales need not match travel.
    const float correction = fastProfile
        ? constrain(f.yawDegrees * 5.0f, -12.0f, 12.0f)
        : constrain(progressError * 0.5f, -4.0f, 4.0f) +
          constrain(f.yawDegrees * 3.0f, -8.0f, 8.0f);
    const float leftTarget = fmaxf(0.0f, baseSpeed + correction);
    const float rightTarget = fmaxf(0.0f, baseSpeed - correction);
    // A leading wheel must not be braked against the IMU heading correction.
    // Positive yaw needs left faster; negative yaw needs right faster.
    // Gross encoder disagreement still terminates the whole trial.
    if (fabsf(progressError) > 20.0f) {
        beginForwardBrake("diferenca entre rodas >20mm", now, left, right);
        return;
    }
    bool brakeLeft = !fastProfile && progressError < -6.0f && f.yawDegrees <= 0.5f;
    bool brakeRight = !fastProfile && progressError > 6.0f && f.yawDegrees >= -0.5f;
    const float leftError = leftTarget - f.leftSpeed;
    const float rightError = rightTarget - f.rightSpeed;
    if (!brakeLeft) f.leftIntegral = constrain(f.leftIntegral + 5.0f * leftError * dt, -80.0f, 100.0f);
    if (!brakeRight) f.rightIntegral = constrain(f.rightIntegral + 5.0f * rightError * dt, -80.0f, 100.0f);
    const int leftPwm = static_cast<int>(constrain(90.0f + 0.4f * leftTarget + 1.5f * leftError + f.leftIntegral, 0.0f, 220.0f));
    const int rightPwm = static_cast<int>(constrain(90.0f + 0.4f * rightTarget + 1.5f * rightError + f.rightIntegral, 0.0f, 220.0f));
    motors.setTurnOutputs(leftPwm, rightPwm, brakeLeft || leftTarget <= 0.0f || leftPwm == 0, brakeRight || rightTarget <= 0.0f || rightPwm == 0);
    if (static_cast<uint32_t>(now - f.lastLogMs) >= 100) {
        f.lastLogMs = now;
        Serial.printf("AVANCO RODAS: L=%.1fmm R=%.1fmm yaw=%.2f pwm=%d/%d freio=%u/%u\n", leftMm, rightMm, f.yawDegrees, leftPwm, rightPwm, static_cast<unsigned>(brakeLeft), static_cast<unsigned>(brakeRight));
    }
}

void updateStationaryIMU(uint32_t now) {
    if (!stationaryActive || static_cast<uint32_t>(now - stationaryLastMs) < 5) return;
    stationaryLastMs = now;
    MPU6500Reading reading{};
    int32_t left = 0, right = 0;
    if (!imu.read(reading) || !leftEncoder.readCount(left) || !rightEncoder.readCount(right)) {
        stopAll();
        Serial.println("IMU PARADA: falha de leitura; teste cancelado.");
        return;
    }
    if (left != stationaryStartLeft || right != stationaryStartRight) {
        stopAll();
        Serial.println("IMU PARADA: encoder mudou; manter robo imovel e repetir.");
        return;
    }
    stationaryGyroSum += reading.gyroZdps;
    stationaryPeakGyro = fmaxf(stationaryPeakGyro, fabsf(reading.gyroZdps));
    ++stationarySamples;
    if (static_cast<uint32_t>(now - stationaryLogMs) >= 1000) {
        stationaryLogMs = now;
        Serial.printf("IMU PARADA: t=%.1fs yaw=%.3f gyro=%.3fdps\n",
                      (now - stationaryStartMs) / 1000.0f, reading.yawDegrees, reading.gyroZdps);
    }
    if (static_cast<uint32_t>(now - stationaryStartMs) >= 10000) {
        stopAll();
        Serial.printf("IMU PARADA FIM: yaw=%.3f mediaGyro=%.4fdps pico=%.3fdps amostras=%lu; encoders imoveis; motores desabilitados.\n",
                      reading.yawDegrees, stationaryGyroSum / stationarySamples, stationaryPeakGyro,
                      static_cast<unsigned long>(stationarySamples));
    }
}

void printDistance(const char* label, UltrasonicUS100& sensor) {
    const UltrasonicReading reading = sensor.measure();
    if (reading.valid) Serial.printf("%s=%.2f cm\n", label, reading.distanceCm);
    else Serial.printf("%s=INVALIDO pulse_us=%lu (nao inferir caminho livre)\n", label,
                       static_cast<unsigned long>(reading.pulseUs));
}

void help() {
    Serial.println("PISO V17: i=calibrar; d=IMU parada 10s; b=armar; f=avanco 30mm; j=180mm lento; k=180mm 60mm/s; n=180mm 80mm/s; m=180mm 100mm/s; t=30 direita; q=90 direita; e=90 esquerda; u=180 direita; w=90 direita 45dps; o=180 direita 45dps; y=90 direita 55dps; p=180 direita 55dps.");
    Serial.println("BANCADA GUIADA: sem partida automatica, sem navegacao.");
    Serial.println("v=290 contagens esquerda; g=290 direita; estimativa de uma volta, com freio ate repouso (200..800 ms) e timeout de 2 s");
    Serial.println("i=inicializar/calibrar IMU imovel; s=distancias; c=contagens; z=zerar encoders");
    Serial.println("SOMENTE COM RODAS SUSPENSAS: a=armar por 10 s; l/r=pulso unico de 200 ms na roda L/R");
    Serial.println("x=parar/desarmar; h=ajuda. Cada pulso exige novo a. LEDC PWM fixo=230/255; wake=10 ms.");
}

void setup() {
    motorsReady = motors.begin();
    stopAll();
    Serial.begin(115200);
    delay(300);
    const bool leftReady = leftEncoder.begin(BoardPins::ENCODER_LEFT_A, BoardPins::ENCODER_LEFT_B,
        BoardPins::ENCODER_LEFT_INVERT_DIRECTION);
    const bool rightReady = rightEncoder.begin(BoardPins::ENCODER_RIGHT_A, BoardPins::ENCODER_RIGHT_B,
        BoardPins::ENCODER_RIGHT_INVERT_DIRECTION);
    encodersReady = leftReady && rightReady;
    frontSensor.begin(BoardPins::ULTRASONIC_FRONT.trigger, BoardPins::ULTRASONIC_FRONT.echo, 20000);
    leftSensor.begin(BoardPins::ULTRASONIC_LEFT.trigger, BoardPins::ULTRASONIC_LEFT.echo, 20000);
    rightSensor.begin(BoardPins::ULTRASONIC_RIGHT.trigger, BoardPins::ULTRASONIC_RIGHT.echo, 20000);
    help();
    Serial.printf("Drivers motores=%s encoders=%s; EEP GPIO5=LOW\n",
                  motorsReady ? "OK" : "FALHA", encodersReady ? "OK" : "FALHA");
}

void loop() {
    const uint32_t now = millis();
    updateGuidedTurn(now);
    updateGuidedForward(now);
    updateStationaryIMU(now);
    if (rotationActive) {
        if (rotationBraking) {
            int32_t count = 0;
            EncoderN20& encoder = rotationLeft ? leftEncoder : rightEncoder;
            if (!encoder.readCount(count)) {
                stopAll();
                Serial.println("FALHA: leitura encoder durante frenagem.");
                return;
            }
            if (count != rotationBrakeLastCount) {
                rotationBrakeLastCount = count;
                rotationQuietSinceMs = now;
            }
            const bool settled = static_cast<uint32_t>(now - rotationBrakingMs) >= 200 &&
                                 static_cast<uint32_t>(now - rotationQuietSinceMs) >= 80;
            const bool brakeTimeout = static_cast<uint32_t>(now - rotationBrakingMs) >= 800;
            if (settled || brakeTimeout) {
                stopAll();
                Serial.println(settled ? "ROTACAO CONCLUIDA; repouso confirmado pelo encoder; driver desabilitado."
                                       : "FALHA: limite de frenagem sem repouso confirmado; driver desabilitado.");
                printCounts();
            }
        } else {
            int32_t count = 0;
            EncoderN20& encoder = rotationLeft ? leftEncoder : rightEncoder;
            if (!encoder.readCount(count)) {
                stopAll();
                Serial.println("ROTACAO FALHOU: leitura encoder.");
                return;
            }
            const int32_t delta = count - rotationStartCount;
            if (delta > rotationPreviousDelta) {
                rotationPreviousDelta = delta;
                rotationProgressMs = now;
            }
            if (delta >= ROTATION_TARGET_COUNTS) {
                motors.brake();
                rotationBraking = true;
                rotationBrakingMs = now;
                rotationQuietSinceMs = now;
                rotationBrakeLastCount = count;
                Serial.printf("ALVO ATINGIDO delta=%ld; freando ate repouso.\n", static_cast<long>(delta));
            } else if (static_cast<uint32_t>(now - rotationStartedMs) >= 2000 ||
                       static_cast<uint32_t>(now - rotationProgressMs) >= 400 || delta < -20) {
                stopAll();
                Serial.printf("ROTACAO INTERROMPIDA: tempo/sem progresso/sinal; delta=%ld\n", static_cast<long>(delta));
                printCounts();
            } else if (static_cast<uint32_t>(now - rotationStartedMs) >= 30) {
                const int pwm = ROTATION_TARGET_COUNTS - delta < 150 ? 100 : 160;
                motors.setSpeeds(rotationLeft ? pwm : 0, rotationLeft ? 0 : pwm);
            }
        }
    }
    if (pulseActive && static_cast<uint32_t>(now - pulseStartedMs) >= 200) {
        stopAll();
        Serial.println("PULSO CONCLUIDO; driver desabilitado.");
        printCounts();
    }
    if (armed && static_cast<uint32_t>(now - armedAtMs) >= 10000) {
        stopAll();
        Serial.println("Armamento expirou.");
    }
    if (!Serial.available()) { delay(1); return; }
    const char command = Serial.read();
    if (command == '\r' || command == '\n' || command == ' ') return;
    if (command == 'x') { stopAll(); Serial.println("PARADO; EEP GPIO5=LOW"); return; }
    // Nenhuma leitura bloqueante durante o pulso, inclusive de sensores.
    if (stationaryActive || pulseActive || rotationActive || turnActive || forwardTrial.active) { Serial.println("Pulso ativo: somente x ou aguardar 200 ms."); return; }
    switch (command) {
        case 'h': help(); break;
        case 'c': printCounts(); break;
        case 'z':
            stopAll();
            if (encodersReady && leftEncoder.resetCount() && rightEncoder.resetCount())
                Serial.println("Encoders zerados.");
            else Serial.println("Falha ao zerar encoders.");
            break;
        case 's':
            stopAll();
            printDistance("FRENTE", frontSensor);
            delay(50);
            printDistance("ESQUERDA", leftSensor);
            delay(50);
            printDistance("DIREITA", rightSensor);
            break;
        case 'i':
            stopAll();
            if (!wireReady) {
                wireReady = Wire.begin(BoardPins::I2C_SDA, BoardPins::I2C_SCL);
                if (wireReady) { Wire.setClock(BoardPins::I2C_CLOCK_HZ); Wire.setTimeOut(20); }
            }
            imuReady = wireReady && imu.begin(Wire, BoardPins::MPU6500_ADDRESS);
            if (!imuReady) { Serial.println("IMU AUSENTE/FALHA. Nenhum giro habilitado."); break; }
            Serial.println("Calibrando IMU: manter imovel.");
            imuReady = imu.calibrateGyroZ();
            Serial.printf("IMU id=0x%02X calibracao=%s\n", imu.deviceId(), imuReady ? "OK" : "FALHA");
            break;
        case 'd':
            stopAll();
            if (!imuReady || !encodersReady || !leftEncoder.readCount(stationaryStartLeft) ||
                !rightEncoder.readCount(stationaryStartRight)) {
                Serial.println("IMU PARADA RECUSADA: requer i e encoders.");
                break;
            }
            imu.resetYaw(0.0f);
            stationaryGyroSum = stationaryPeakGyro = 0.0f;
            stationarySamples = 0;
            stationaryStartMs = stationaryLastMs = stationaryLogMs = millis();
            stationaryActive = true;
            Serial.println("IMU PARADA INICIO: 10s, nao tocar; motores desabilitados; x cancela.");
            break;
        case 'b':
            stopAll();
            if (!motorsReady || !encodersReady || !imuReady) {
                Serial.println("Recusado: inicializar IMU com i e conferir drivers.");
                break;
            }
            armed = true;
            floorArmed = true;
            armedAtMs = millis();
            Serial.println("PISO ARMADO por 10 s: f=30mm; j=180mm; k=180mm 60mm/s; n=180mm 80mm/s; m=180mm 100mm/s; t/q/e/u=giros lentos; w/o=90/180 direita 45dps; y/p=90/180 direita 55dps; x cancela.");
            break;
        case 'f':
        case 'j':
        case 'k':
        case 'n':
        case 'm': {
            const bool fastTrial = command == 'k' || command == 'n' || command == 'm';
            const float targetMm = command == 'f' ? 30.0f : 180.0f;
            const float minimumFrontCm = targetMm / 10.0f + 6.0f;
            if (!armed || !floorArmed || !imuReady) { Serial.println("Recusado: requer i e b."); break; }
            const UltrasonicReading front = frontSensor.measure();
            MPU6500Reading reading{};
            int32_t left = 0, right = 0;
            if (!front.valid || front.distanceCm < minimumFrontCm || !imu.read(reading) ||
                fabsf(reading.gyroZdps) > 4.0f || !leftEncoder.readCount(left) || !rightEncoder.readCount(right)) {
                stopAll();
                Serial.printf("AVANCO RECUSADO: conferir IMU/encoders, robo imovel e frente >=%.1fcm.\n", minimumFrontCm);
                break;
            }
            forwardTrial = ForwardTrial{};
            forwardTrial.targetMm = targetMm;
            forwardTrial.timeoutMs = fastTrial ? 6000 : (command == 'j' ? 10000 : 4000);
            forwardTrial.maximumSpeedMmS = command == 'm' ? 100.0f : (command == 'n' ? 80.0f : (fastTrial ? 60.0f : 30.0f));
            forwardTrial.startLeft = forwardTrial.previousLeft = left;
            forwardTrial.startRight = forwardTrial.previousRight = right;
            imu.resetYaw(0.0f);
            armed = false;
            floorArmed = false;
            motors.setEnabled(true);
            forwardTrial.startedMs = millis();
            forwardTrial.lastUpdateMs = forwardTrial.lastFrontMs = forwardTrial.lastLogMs = forwardTrial.startedMs;
            forwardTrial.leftProgressMs = forwardTrial.rightProgressMs = forwardTrial.startedMs;
            forwardTrial.active = true;
            Serial.printf("AVANCO INICIADO: alvo %.0fmm; frente=%.2fcm; vmax=%.0fmm/s; limite %lus; x interrompe.\n", targetMm, front.distanceCm, forwardTrial.maximumSpeedMmS, static_cast<unsigned long>(forwardTrial.timeoutMs / 1000));
            break;
        }
        case 't':
        case 'q':
        case 'e':
        case 'u':
        case 'w':
        case 'o':
        case 'y':
        case 'p': {
            if (!armed || !floorArmed || !imuReady) { Serial.println("Recusado: requer i e b."); break; }
            MPU6500Reading reading{};
            if (!imu.read(reading) || fabsf(reading.gyroZdps) > 4.0f ||
                !leftEncoder.readCount(turnStartLeft) || !rightEncoder.readCount(turnStartRight)) {
                stopAll();
                Serial.println("Recusado: IMU/encoder ou robo em movimento.");
                break;
            }
            armed = false;
            floorArmed = false;
            imu.resetYaw(0.0f);
            continuousTurnYaw.reset(0.0f);
            turnDirectionSign = command == 'e' ? -1 : 1;
            turnTargetDegrees = (command == 'u' || command == 'o' || command == 'p') ? 180.0f : (command == 't' ? 30.0f : 90.0f);
            turnMaximumRateDps = (command == 'y' || command == 'p') ? 55.0f : ((command == 'w' || command == 'o') ? 45.0f : 35.0f);
            turnTimeoutMs = (command == 'u' || command == 'o' || command == 'p') ? 11000 : (command == 't' ? 3000 : 6500);
            turnAngleDeg = 0.0f;
            turnProgressDeg = 0.0f;
            turnControlLastLeft = turnStartLeft;
            turnControlLastRight = turnStartRight;
            turnLeftSpeedMmS = 0.0f;
            turnRightSpeedMmS = 0.0f;
            turnLeftIntegralPwm = 0.0f;
            turnRightIntegralPwm = 0.0f;
            turnLastLogMs = millis();
            motors.setEnabled(true);
            turnStartedMs = millis();
            turnLastUpdateMs = turnStartedMs;
            turnLastProgressMs = turnStartedMs;
            turnActive = true;
            turnBraking = false;
            Serial.printf("GIRO INICIADO: %s %.0f graus; IMU 5 ms; limite %lu ms; x interrompe.\n",
                          turnDirectionSign > 0 ? "direita" : "esquerda",
                          turnTargetDegrees, static_cast<unsigned long>(turnTimeoutMs));
            break;
        }
        case 'a':
            stopAll();
            if (!motorsReady || !encodersReady) { Serial.println("Armamento recusado: falha de drivers."); break; }
            armed = true;
            armedAtMs = millis();
            Serial.println("ARMADO por 10 s. l/r SOMENTE com rodas suspensas.");
            break;
        case 'v':
        case 'g': {
            if (!armed || floorArmed) { Serial.println("Recusado: armar com a, rodas suspensas."); break; }
            rotationLeft = command == 'v';
            EncoderN20& encoder = rotationLeft ? leftEncoder : rightEncoder;
            if (!encoder.readCount(rotationStartCount)) { stopAll(); break; }
            armed = false;
            rotationPreviousDelta = 0;
            motors.setEnabled(true);
            rotationStartedMs = millis();
            rotationProgressMs = rotationStartedMs;
            rotationActive = true;
            rotationBraking = false;
            motors.setSpeeds(rotationLeft ? 230 : 0, rotationLeft ? 0 : 230);
            Serial.println("ROTACAO 290 contagens: boost 30 ms; PWM 160/100; limite 2 s.");
            break;
        }
        case 'l':
        case 'r':
            if (!armed || floorArmed) { Serial.println("Recusado: armar com a, rodas suspensas."); break; }
            armed = false;
            motors.setEnabled(true);
            motors.setSpeeds(command == 'l' ? 230 : 0, command == 'r' ? 230 : 0);
            pulseStartedMs = millis();
            pulseActive = true;
            Serial.println("PULSO 200 ms");
            break;
        default: Serial.println("Comando desconhecido; h=ajuda."); break;
    }
}
