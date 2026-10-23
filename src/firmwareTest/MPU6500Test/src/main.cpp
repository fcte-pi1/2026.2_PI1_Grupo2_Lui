#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "i2c_diagnostic.hpp"
#include "diagnostic_logger.hpp"
#include "../../../firmware/include/config/board_pins.hpp"
#include "../../../firmware/src/hal/imu_mpu6500.hpp"

namespace {
IMUMPU6500 imu;
uint8_t activeAddress = BoardPins::MPU6500_ADDRESS;
constexpr unsigned SAMPLE_COUNT = 20;

bool locateDevice() {
    const uint8_t candidates[] = {BoardPins::MPU6500_ADDRESS,
                                  static_cast<uint8_t>(BoardPins::MPU6500_ADDRESS == 0x68 ? 0x69 : 0x68)};
    for (uint8_t address : candidates) {
        uint8_t status = 0xFF;
        const bool ack = DiagnosticI2C::probe(Wire, address, status);
        DiagnosticLog::log(ack ? "PASS" : "WARN", "PRE-CHECK I2C endereco 0x%02X: %s (status endTransmission=%u).",
            address, ack ? "ACK recebido" : "sem ACK", status);
        if (ack) {
            activeAddress = address;
            return true;
        }
    }
    return false;
}
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.setTimeout(30000);
    DiagnosticLog::banner("IMU MPU6500");
    DiagnosticLog::log("INFO", "Barramento I2C: SDA=GPIO%u, SCL=GPIO%u, clock=%lu Hz.",
        BoardPins::I2C_SDA, BoardPins::I2C_SCL, static_cast<unsigned long>(BoardPins::I2C_CLOCK_HZ));
    if (!Wire.begin(BoardPins::I2C_SDA, BoardPins::I2C_SCL)) {
        DiagnosticLog::log("ERROR", "Barramento I2C nao inicializou; nenhuma leitura da IMU sera tentada.");
        return;
    }
    Wire.setClock(BoardPins::I2C_CLOCK_HZ);

    DiagnosticLog::log("STAGE", "FASE 1 - teste de conexao I2C. Nenhuma calibracao/leitura funcional ocorrera antes do ACK e validacao da identificacao.");
    if (!locateDevice()) {
        DiagnosticLog::log("ERROR", "MPU6500 nao respondeu em 0x68 nem 0x69. Diagnostico funcional BLOQUEADO.");
        DiagnosticLog::log("WARN", "Verifique 3V3, GND, SDA/SCL, pull-ups e pino AD0; confirme se o modulo usa MPU6500 ou variante compativel.");
        return;
    }

    DiagnosticLog::log("INFO", "Dispositivo respondeu em 0x%02X; chamando o driver para validar WHO_AM_I e configurar registradores.", activeAddress);
    if (!imu.begin(Wire, activeAddress)) {
        DiagnosticLog::log("ERROR", "I2C respondeu em 0x%02X, mas WHO_AM_I/initializacao falhou. Conexao eletrica existe, identidade funcional nao validada; restante bloqueado.", activeAddress);
        return;
    }
    DiagnosticLog::log("PASS", "PRE-CHECK COMPLETO: WHO_AM_I=0x%02X aceito pelo driver; MPU inicializada em 0x%02X.", imu.deviceId(), activeAddress);

    DiagnosticLog::log("STAGE", "FASE 2 - calibracao do giroscopio e diagnostico funcional liberados apos conexao confirmada.");
    if (!DiagnosticLog::answerYes("Deixe o robo parado, nivelado e com motores DESABILITADOS. Esta condicao esta garantida?")) {
        DiagnosticLog::log("WARN", "Calibracao/leitura funcional cancelada pelo operador.");
        return;
    }
    DiagnosticLog::log("INFO", "Calibrando gyro Z com 300 amostras (intervalo 3 ms). Nao mova o modulo.");
    if (!imu.calibrateGyroZ(300, 3)) {
        DiagnosticLog::log("ERROR", "Falha na calibracao do gyro Z; revisar conexao/ruido e nao confiar na estimativa angular.");
        return;
    }
    DiagnosticLog::log("PASS", "Calibracao do gyro Z concluida.");

    unsigned valid = 0;
    float accelMagnitudeSum = 0.0f;
    float gyroZAbsSum = 0.0f;
    for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
        MPU6500Reading reading{};
        if (!imu.read(reading)) {
            DiagnosticLog::log("ERROR", "Amostra %u/%u: falha ao ler frame por I2C.", i + 1, SAMPLE_COUNT);
            delay(100);
            continue;
        }
        const float magnitude = sqrtf(reading.accelXg * reading.accelXg +
                                      reading.accelYg * reading.accelYg +
                                      reading.accelZg * reading.accelZg);
        if (!isfinite(magnitude) || !isfinite(reading.gyroZdps) || !isfinite(reading.yawDegrees)) {
            DiagnosticLog::log("ERROR", "Amostra %u/%u: valor NaN/inf detectado.", i + 1, SAMPLE_COUNT);
            delay(100);
            continue;
        }
        ++valid;
        accelMagnitudeSum += magnitude;
        gyroZAbsSum += fabsf(reading.gyroZdps);
        DiagnosticLog::log("INFO", "Amostra %u/%u: accel[g]=(%+.3f,%+.3f,%+.3f), |a|=%.3f g; gyro[dps]=(%+.2f,%+.2f,%+.2f); yaw_relativo=%+.2f deg.",
            i + 1, SAMPLE_COUNT, reading.accelXg, reading.accelYg, reading.accelZg,
            magnitude, reading.gyroXdps, reading.gyroYdps, reading.gyroZdps, reading.yawDegrees);
        delay(100);
    }
    DiagnosticLog::log("INFO", "Resumo MPU6500: amostras validas=%u/%u.", valid, SAMPLE_COUNT);
    if (valid == SAMPLE_COUNT) {
        const float avgAccel = accelMagnitudeSum / valid;
        const float avgGyroAbs = gyroZAbsSum / valid;
        DiagnosticLog::log("INFO", "Media |aceleracao|=%.3f g; media |gyro Z|=%.3f dps.", avgAccel, avgGyroAbs);
        if (avgAccel >= 0.70f && avgAccel <= 1.30f) {
            DiagnosticLog::log("PASS", "Magnitude de aceleracao plausivel (~1 g); leitura basica funcional aprovada.");
        } else {
            DiagnosticLog::log("WARN", "Magnitude de aceleracao fora do intervalo esperado 0.70-1.30 g; verifique montagem, nivelamento e escala do sensor.");
        }
        if (avgGyroAbs <= 5.0f) DiagnosticLog::log("PASS", "Gyro Z dentro de 5 dps medios durante o teste parado.");
        else DiagnosticLog::log("WARN", "Gyro Z acima de 5 dps medios: o modulo pode ter se movido ou exigir verificacao de ruido/calibracao.");
    } else {
        DiagnosticLog::log("ERROR", "Menos de 100%% das leituras foram validas; revisar I2C e estabilidade do modulo.");
    }
    DiagnosticLog::log("INFO", "O yaw reportado e orientacao relativa integrada, nao um norte absoluto.");
}

void loop() {
    delay(1000);
}
