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
constexpr unsigned IDLE_SAMPLE_COUNT = 20;
constexpr uint32_t CAPTURE_SAMPLE_INTERVAL_MS = 10;
constexpr uint32_t CAPTURE_LOG_INTERVAL_MS = 100;
constexpr uint32_t CAPTURE_TIMEOUT_MS = 15000;
constexpr float TARGET_TURN_DEGREES = 90.0f;
constexpr float TURN_TOLERANCE_DEGREES = 30.0f;
constexpr float RETURN_TOLERANCE_DEGREES = 20.0f;

struct RotationCapture {
    bool hasData = false;
    bool stoppedByOperator = false;
    bool timedOut = false;
    bool readFailed = false;
    float startYaw = 0.0f;
    float endYaw = 0.0f;
    float deltaYaw = 0.0f;
    unsigned validSamples = 0;
    unsigned failedSamples = 0;
    uint32_t durationMs = 0;
};

float wrapDegrees(float angle) {
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

bool locateDevice() {
    const uint8_t candidates[] = {
        BoardPins::MPU6500_ADDRESS,
        static_cast<uint8_t>(BoardPins::MPU6500_ADDRESS == 0x68 ? 0x69 : 0x68)
    };
    for (uint8_t address : candidates) {
        uint8_t status = 0xFF;
        const bool ack = DiagnosticI2C::probe(Wire, address, status);
        DiagnosticLog::log(ack ? "PASS" : "WARN",
            "PRE-CHECK I2C endereco 0x%02X: %s (status endTransmission=%u).",
            address, ack ? "ACK recebido" : "sem ACK", status);
        if (ack) {
            activeAddress = address;
            return true;
        }
    }
    return false;
}

// Returns true when the monitor sends a line ending. It handles LF, CR, and CRLF.
bool consumeEnterFromSerial() {
    bool enterPressed = false;
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value == '\n' || value == '\r') enterPressed = true;
    }
    return enterPressed;
}

RotationCapture captureRotation(const char* readyInstruction,
                                const char* activeInstruction,
                                bool resetYawToZero) {
    RotationCapture result;
    DiagnosticLog::waitForEnter(readyInstruction);

    // Refresh the driver's time base immediately before acquisition. For the
    // return leg, preserve the accumulated yaw instead of starting at zero.
    const float preservedYaw = resetYawToZero ? 0.0f : imu.getYawDegrees();
    imu.resetYaw(preservedYaw);

    MPU6500Reading reading{};
    if (!imu.read(reading)) {
        result.readFailed = true;
        result.failedSamples = 1;
        DiagnosticLog::log("ERROR", "Captura dinamica abortada: falha na primeira leitura da IMU.");
        return result;
    }

    result.startYaw = reading.yawDegrees;
    result.endYaw = reading.yawDegrees;
    result.hasData = true;
    ++result.validSamples;
    uint32_t lastLogMs = millis();
    const uint32_t startedMs = millis();

    DiagnosticLog::log("ACTION", "%s", activeInstruction);
    DiagnosticLog::log("INFO", "Captura ativa: pressione ENTER novamente ao concluir o movimento (limite %lu s).",
        static_cast<unsigned long>(CAPTURE_TIMEOUT_MS / 1000));

    while (millis() - startedMs < CAPTURE_TIMEOUT_MS) {
        if (consumeEnterFromSerial()) {
            result.stoppedByOperator = true;
            break;
        }

        if (!imu.read(reading)) {
            ++result.failedSamples;
            result.readFailed = true;
            DiagnosticLog::log("ERROR", "Falha de leitura I2C durante a captura dinamica; etapa interrompida.");
            break;
        }

        ++result.validSamples;
        result.endYaw = reading.yawDegrees;
        const float delta = wrapDegrees(result.endYaw - result.startYaw);

        const uint32_t nowMs = millis();
        if (nowMs - lastLogMs >= CAPTURE_LOG_INTERVAL_MS) {
            DiagnosticLog::log("INFO", "Captura yaw=%+.2f deg; delta=%+.2f deg; gyroZ=%+.2f dps; amostras=%u.",
                result.endYaw, delta, reading.gyroZdps, result.validSamples);
            lastLogMs = nowMs;
        }
        delay(CAPTURE_SAMPLE_INTERVAL_MS);
    }

    if (!result.stoppedByOperator && !result.readFailed && millis() - startedMs >= CAPTURE_TIMEOUT_MS) {
        result.timedOut = true;
        DiagnosticLog::log("WARN", "Tempo limite da captura atingido; usando as amostras obtidas ate aqui.");
    }

    // Include one last reading so the final angle reflects the end of the motion.
    if (!result.readFailed && imu.read(reading)) {
        ++result.validSamples;
        result.endYaw = reading.yawDegrees;
    } else if (!result.readFailed) {
        result.readFailed = true;
        ++result.failedSamples;
        DiagnosticLog::log("ERROR", "Falha na leitura final da captura dinamica.");
    }

    result.durationMs = millis() - startedMs;
    result.deltaYaw = wrapDegrees(result.endYaw - result.startYaw);
    DiagnosticLog::log("INFO", "Fim da captura: yaw inicial=%+.2f deg; final=%+.2f deg; delta=%+.2f deg; duracao=%lu ms; leituras validas=%u; falhas=%u.",
        result.startYaw, result.endYaw, result.deltaYaw,
        static_cast<unsigned long>(result.durationMs), result.validSamples, result.failedSamples);
    return result;
}

struct AxisPose {
    const char* label;
    const char* instruction;
    float expectedX;
    float expectedY;
    float expectedZ;
};

bool testAxisPose(const AxisPose& pose) {
    constexpr unsigned ORIENTATION_SAMPLES = 20;
    constexpr uint32_t ORIENTATION_SAMPLE_INTERVAL_MS = 25;
    constexpr float MIN_AXIS_G = 0.70f;
    constexpr float MAX_AXIS_G = 1.30f;
    constexpr float MAX_CROSS_AXIS_G = 0.35f;

    DiagnosticLog::waitForEnter(pose.instruction);
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;
    unsigned valid = 0;

    for (unsigned i = 0; i < ORIENTATION_SAMPLES; ++i) {
        MPU6500Reading reading{};
        if (!imu.read(reading) || !isfinite(reading.accelXg) ||
            !isfinite(reading.accelYg) || !isfinite(reading.accelZg)) {
            DiagnosticLog::log("ERROR", "Pose %s: falha de leitura/valor invalido na amostra %u/%u.",
                pose.label, i + 1, ORIENTATION_SAMPLES);
            delay(ORIENTATION_SAMPLE_INTERVAL_MS);
            continue;
        }
        sumX += reading.accelXg;
        sumY += reading.accelYg;
        sumZ += reading.accelZg;
        ++valid;
        delay(ORIENTATION_SAMPLE_INTERVAL_MS);
    }

    if (valid < ORIENTATION_SAMPLES * 4 / 5) {
        DiagnosticLog::log("ERROR", "Pose %s: somente %u/%u leituras validas; resultado inconclusivo.",
            pose.label, valid, ORIENTATION_SAMPLES);
        return false;
    }

    const float avgX = sumX / valid;
    const float avgY = sumY / valid;
    const float avgZ = sumZ / valid;
    const float magnitude = sqrtf(avgX * avgX + avgY * avgY + avgZ * avgZ);
    DiagnosticLog::log("INFO", "Pose %s: accel medio=(%+.3f,%+.3f,%+.3f) g; |a|=%.3f g; leituras=%u/%u.",
        pose.label, avgX, avgY, avgZ, magnitude, valid, ORIENTATION_SAMPLES);

    const bool dominantAxisCorrect =
        (pose.expectedX == 0.0f || (avgX * pose.expectedX >= MIN_AXIS_G)) &&
        (pose.expectedY == 0.0f || (avgY * pose.expectedY >= MIN_AXIS_G)) &&
        (pose.expectedZ == 0.0f || (avgZ * pose.expectedZ >= MIN_AXIS_G));
    const bool crossAxesSmall =
        (pose.expectedX != 0.0f || fabsf(avgX) <= MAX_CROSS_AXIS_G) &&
        (pose.expectedY != 0.0f || fabsf(avgY) <= MAX_CROSS_AXIS_G) &&
        (pose.expectedZ != 0.0f || fabsf(avgZ) <= MAX_CROSS_AXIS_G);
    const bool magnitudePlausible = magnitude >= 0.70f && magnitude <= 1.30f;

    if (dominantAxisCorrect && crossAxesSmall && magnitudePlausible) {
        DiagnosticLog::log("PASS", "Pose %s aprovada: eixo/sinal esperado detectado.", pose.label);
        return true;
    }

    DiagnosticLog::log("WARN", "Pose %s fora da tolerancia. Esperado aproximadamente (%+.0f,%+.0f,%+.0f) g; confira orientacao, nivelamento e mapeamento dos eixos.",
        pose.label, pose.expectedX, pose.expectedY, pose.expectedZ);
    return false;
}

void runAccelerometerAxisOrientationTest() {
    DiagnosticLog::log("STAGE", "FASE 3 - teste de orientacao dos eixos X/Y/Z usando o ACELEROMETRO.");
    DiagnosticLog::log("INFO", "O giroscopio mede velocidade angular; a orientacao estatica dos eixos e verificada pela gravidade no acelerometro.");
    DiagnosticLog::log("INFO", "Em cada pose, deixe o modulo parado, com o eixo indicado apontando verticalmente para cima, e pressione ENTER.");
    DiagnosticLog::log("INFO", "Vetor esperado em g: +X=(+1,0,0), -X=(-1,0,0), +Y=(0,+1,0), -Y=(0,-1,0), +Z=(0,0,+1), -Z=(0,0,-1).");

    const AxisPose poses[] = {
        {"+X para cima", "Pose 1/6: coloque o modulo com o eixo +X apontando para CIMA e mantenha-o parado. Pressione ENTER para medir.", +1.0f, 0.0f, 0.0f},
        {"-X para cima", "Pose 2/6: gire o modulo para que o eixo -X aponte para CIMA. Mantenha-o parado e pressione ENTER.", -1.0f, 0.0f, 0.0f},
        {"+Y para cima", "Pose 3/6: coloque o eixo +Y apontando para CIMA. Mantenha-o parado e pressione ENTER.", 0.0f, +1.0f, 0.0f},
        {"-Y para cima", "Pose 4/6: coloque o eixo -Y apontando para CIMA. Mantenha-o parado e pressione ENTER.", 0.0f, -1.0f, 0.0f},
        {"+Z para cima", "Pose 5/6: coloque o eixo +Z apontando para CIMA (orientacao nivelada habitual). Mantenha-o parado e pressione ENTER.", 0.0f, 0.0f, +1.0f},
        {"-Z para cima", "Pose 6/6: vire o modulo para que o eixo -Z aponte para CIMA. Mantenha-o parado e pressione ENTER.", 0.0f, 0.0f, -1.0f}
    };

    unsigned passed = 0;
    for (const AxisPose& pose : poses) {
        if (testAxisPose(pose)) ++passed;
    }
    DiagnosticLog::log(passed == 6 ? "PASS" : "WARN",
        "Resumo orientacao dos eixos: %u/6 poses aprovadas. Tolerancia: eixo esperado >= %.2f g, eixos transversais <= %.2f g, magnitude 0.70-1.30 g.",
        passed, 0.70f, 0.35f);
    if (passed < 6) {
        DiagnosticLog::log("WARN", "Falha em uma pose pode indicar que o eixo nao ficou vertical, modulo se moveu durante a amostragem, ou orientacao/mapeamento dos eixos difere do esperado.");
    }
}

void runDynamicRotationTest() {
    DiagnosticLog::log("STAGE", "FASE 3 - teste dinamico do yaw: giro de ~90 graus e retorno a orientacao inicial.");
    DiagnosticLog::log("INFO", "Mantenha a IMU nivelada e os motores DESABILITADOS; gire o robo manualmente em torno do eixo vertical.");
    DiagnosticLog::log("INFO", "Convencao esperada: anti-horario visto de cima gera delta positivo se o eixo +Z da IMU aponta para cima.");

    const RotationCapture outbound = captureRotation(
        "Giro 1: posicione o robo na referencia inicial. Pressione ENTER para iniciar a captura; depois gire cerca de 90 graus para a ESQUERDA (anti-horario visto de cima) e pressione ENTER para parar.",
        "GO: gire agora cerca de 90 graus para a ESQUERDA (anti-horario visto de cima), mantendo o modulo nivelado.",
        true);

    if (outbound.readFailed || !outbound.hasData) {
        DiagnosticLog::log("ERROR", "Teste dinamico interrompido na primeira etapa; nao e possivel avaliar a orientacao.");
        return;
    }

    const float outboundMagnitude = fabsf(outbound.deltaYaw);
    if (outboundMagnitude >= TARGET_TURN_DEGREES - TURN_TOLERANCE_DEGREES &&
        outboundMagnitude <= TARGET_TURN_DEGREES + TURN_TOLERANCE_DEGREES) {
        DiagnosticLog::log("PASS", "Magnitude do primeiro giro proxima de 90 graus: |delta|=%.2f deg (tolerancia +/-%.0f deg).",
            outboundMagnitude, TURN_TOLERANCE_DEGREES);
    } else {
        DiagnosticLog::log("WARN", "Giro distante de 90 graus: delta=%+.2f deg; esperado aproximadamente +/-%.0f deg. Gire de forma mais uniforme e termine a captura no ponto correto.",
            outbound.deltaYaw, TARGET_TURN_DEGREES);
    }

    if (outbound.deltaYaw > 0.0f) {
        DiagnosticLog::log("PASS", "Giro anti-horario produziu yaw positivo, conforme convencao com eixo +Z apontando para cima.");
    } else if (outbound.deltaYaw < 0.0f) {
        DiagnosticLog::log("WARN", "Giro anti-horario produziu yaw negativo. Confira a orientacao fisica da IMU e imuYawSign usado pelo MotionController.");
    } else {
        DiagnosticLog::log("WARN", "Yaw quase nao mudou durante o primeiro giro; confirme que o robo foi girado e que a IMU esta firmemente presa.");
    }

    const RotationCapture returning = captureRotation(
        "Giro 2: agora retorne o robo a orientacao inicial, girando para a DIREITA (sentido horario visto de cima). Pressione ENTER para iniciar a captura; gire ate a referencia inicial e pressione ENTER para parar.",
        "GO: retorne agora a orientacao inicial girando para a DIREITA (sentido horario visto de cima), mantendo o modulo nivelado.",
        false);

    if (returning.readFailed || !returning.hasData) {
        DiagnosticLog::log("ERROR", "Teste dinamico interrompido na etapa de retorno; fechamento angular nao pode ser avaliado.");
        return;
    }

    if (outbound.deltaYaw * returning.deltaYaw < 0.0f) {
        DiagnosticLog::log("PASS", "O retorno ocorreu com sinal oposto ao primeiro giro (ida=%+.2f deg; retorno=%+.2f deg).",
            outbound.deltaYaw, returning.deltaYaw);
    } else {
        DiagnosticLog::log("WARN", "O sinal do retorno nao foi oposto ao giro inicial (ida=%+.2f deg; retorno=%+.2f deg); confira o movimento e a orientacao do modulo.",
            outbound.deltaYaw, returning.deltaYaw);
    }

    const float closureError = fabsf(wrapDegrees(returning.endYaw));
    DiagnosticLog::log("INFO", "Erro de fechamento em relacao ao yaw inicial (0 deg): %.2f deg.", closureError);
    if (closureError <= RETURN_TOLERANCE_DEGREES) {
        DiagnosticLog::log("PASS", "Yaw retornou proximo da referencia inicial: erro=%.2f deg (limite %.0f deg).",
            closureError, RETURN_TOLERANCE_DEGREES);
    } else {
        DiagnosticLog::log("WARN", "Yaw nao fechou dentro da tolerancia: erro=%.2f deg (limite %.0f deg). Verifique o retorno manual, bias do gyro e fixacao da IMU.",
            closureError, RETURN_TOLERANCE_DEGREES);
    }

    if (outbound.timedOut || returning.timedOut) {
        DiagnosticLog::log("WARN", "Pelo menos uma captura atingiu o tempo limite; considere a avaliacao dinamica inconclusiva.");
    }
    if (outbound.failedSamples > 0 || returning.failedSamples > 0) {
        DiagnosticLog::log("WARN", "Houve falhas de leitura durante a captura; nao use estes resultados como validacao definitiva.");
    }
    DiagnosticLog::log("INFO", "Este teste avalia yaw relativo por integracao do gyro Z; nao representa norte absoluto e o resultado depende da montagem e da execucao manual.");
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
        DiagnosticLog::log("ERROR", "I2C respondeu em 0x%02X, mas WHO_AM_I/inicializacao falhou. Conexao eletrica existe, identidade funcional nao validada; restante bloqueado.", activeAddress);
        return;
    }
    DiagnosticLog::log("PASS", "PRE-CHECK COMPLETO: WHO_AM_I=0x%02X aceito pelo driver; MPU inicializada em 0x%02X.", imu.deviceId(), activeAddress);

    DiagnosticLog::log("STAGE", "FASE 2 - calibracao do giroscopio e verificacao estatica apos conexao confirmada.");
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
    for (unsigned i = 0; i < IDLE_SAMPLE_COUNT; ++i) {
        MPU6500Reading reading{};
        if (!imu.read(reading)) {
            DiagnosticLog::log("ERROR", "Amostra estatica %u/%u: falha ao ler frame por I2C.", i + 1, IDLE_SAMPLE_COUNT);
            delay(100);
            continue;
        }
        const float magnitude = sqrtf(reading.accelXg * reading.accelXg +
                                      reading.accelYg * reading.accelYg +
                                      reading.accelZg * reading.accelZg);
        if (!isfinite(magnitude) || !isfinite(reading.gyroZdps) || !isfinite(reading.yawDegrees)) {
            DiagnosticLog::log("ERROR", "Amostra estatica %u/%u: valor NaN/inf detectado.", i + 1, IDLE_SAMPLE_COUNT);
            delay(100);
            continue;
        }
        ++valid;
        accelMagnitudeSum += magnitude;
        gyroZAbsSum += fabsf(reading.gyroZdps);
        DiagnosticLog::log("INFO", "Amostra estatica %u/%u: accel[g]=(%+.3f,%+.3f,%+.3f), |a|=%.3f g; gyro[dps]=(%+.2f,%+.2f,%+.2f); yaw_relativo=%+.2f deg.",
            i + 1, IDLE_SAMPLE_COUNT, reading.accelXg, reading.accelYg, reading.accelZg,
            magnitude, reading.gyroXdps, reading.gyroYdps, reading.gyroZdps, reading.yawDegrees);
        delay(100);
    }

    DiagnosticLog::log("INFO", "Resumo MPU6500: amostras estaticas validas=%u/%u.", valid, IDLE_SAMPLE_COUNT);
    if (valid == IDLE_SAMPLE_COUNT) {
        const float avgAccel = accelMagnitudeSum / valid;
        const float avgGyroAbs = gyroZAbsSum / valid;
        DiagnosticLog::log("INFO", "Media |aceleracao|=%.3f g; media |gyro Z|=%.3f dps.", avgAccel, avgGyroAbs);
        if (avgAccel >= 0.70f && avgAccel <= 1.30f) {
            DiagnosticLog::log("PASS", "Magnitude de aceleracao plausivel (~1 g); leitura estatica funcional aprovada.");
        } else {
            DiagnosticLog::log("WARN", "Magnitude de aceleracao fora do intervalo esperado 0.70-1.30 g; verifique montagem, nivelamento e escala do sensor.");
        }
        if (avgGyroAbs <= 5.0f) {
            DiagnosticLog::log("PASS", "Gyro Z dentro de 5 dps medios durante o teste parado.");
        } else {
            DiagnosticLog::log("WARN", "Gyro Z acima de 5 dps medios: o modulo pode ter se movido ou exigir verificacao de ruido/calibracao.");
        }
    } else {
        DiagnosticLog::log("ERROR", "Menos de 100%% das leituras estaticas foram validas; revisar I2C e estabilidade do modulo.");
        DiagnosticLog::log("WARN", "O teste dinamico ainda sera oferecido, mas nao trate a avaliacao como aprovada se houver falhas de comunicacao.");
    }

    runAccelerometerAxisOrientationTest();
    runDynamicRotationTest();
    DiagnosticLog::log("INFO", "Diagnostico MPU6500 concluido. O yaw e uma orientacao relativa integrada, nao um norte absoluto.");
}

void loop() {
    delay(1000);
}
