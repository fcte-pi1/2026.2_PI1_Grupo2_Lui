#include <Arduino.h>
#include "diagnostic_logger.hpp"
#include "../../../firmware/include/config/board_pins.hpp"
#include "../../../firmware/src/hal/encoder_n20.hpp"

namespace {
EncoderN20 encoderLeft;
EncoderN20 encoderRight;
constexpr uint32_t PRECHECK_WINDOW_MS = 5000;
constexpr uint32_t DIAGNOSTIC_WINDOW_MS = 2500;
bool leftInitialized = false;
bool rightInitialized = false;
bool leftConnected = false;
bool rightConnected = false;

bool countForWindow(EncoderN20& encoder, uint32_t durationMs, int32_t& delta) {
    if (!encoder.resetCount()) return false;
    uint32_t countStart = millis();
    int32_t previous = 0;
    if (!encoder.readCount(previous)) return false;
    while (static_cast<uint32_t>(millis() - countStart) < durationMs) {
        int32_t current = 0;
        if (!encoder.readCount(current)) return false;
        delay(10);
    }
    int32_t finalCount = 0;
    if (!encoder.readCount(finalCount)) return false;
    delta = finalCount;
    return true;
}

bool connectionPreflight(const char* name, EncoderN20& encoder, bool initialized, uint8_t pinA, uint8_t pinB) {
    DiagnosticLog::log("INFO", "Encoder %s: canal A=GPIO%u (PCNT), canal B=GPIO%u (sentido). Driver inicializado=%s.",
        name, pinA, pinB, initialized ? "SIM" : "NAO");
    if (!initialized) {
        DiagnosticLog::log("ERROR", "PRE-CHECK %s: PCNT nao inicializou; conexao nao confirmada e diagnostico bloqueado.", name);
        return false;
    }
    char prompt[180];
    snprintf(prompt, sizeof(prompt), "PRE-CHECK %s: eleve o chassi, mantenha motores DESLIGADOS e gire APENAS a roda %s manualmente durante %lu s; pressione ENTER e comece a girar.",
        name, name, static_cast<unsigned long>(PRECHECK_WINDOW_MS / 1000));
    DiagnosticLog::waitForEnter(prompt);
    int32_t delta = 0;
    if (!countForWindow(encoder, PRECHECK_WINDOW_MS, delta)) {
        DiagnosticLog::log("ERROR", "PRE-CHECK %s: leitura/reset do contador falhou; diagnostico bloqueado.", name);
        return false;
    }
    if (delta == 0) {
        DiagnosticLog::log("ERROR", "PRE-CHECK %s: nenhum pulso registrado em %lu ms. Conexao NAO confirmada; etapas funcionais deste encoder serao ignoradas.",
            name, static_cast<unsigned long>(PRECHECK_WINDOW_MS));
        DiagnosticLog::log("WARN", "Confirme movimento real da roda, alimentacao do encoder, GND comum, fio do canal A e correspondencia dos GPIOs.");
        return false;
    }
    DiagnosticLog::log("PASS", "PRE-CHECK %s: pulsos detectados (delta=%ld em %lu ms). Conexao operacional confirmada para continuar.",
        name, static_cast<long>(delta), static_cast<unsigned long>(PRECHECK_WINDOW_MS));
    return true;
}

void diagnoseDirection(const char* name, EncoderN20& encoder) {
    int32_t forwardDelta = 0;
    int32_t reverseDelta = 0;
    char prompt[180];
    snprintf(prompt, sizeof(prompt), "DIAGNOSTICO %s (1/2): gire a roda %s em um sentido por cerca de %lu s; pressione ENTER e comece.",
        name, name, static_cast<unsigned long>(DIAGNOSTIC_WINDOW_MS / 1000));
    DiagnosticLog::waitForEnter(prompt);
    const bool firstOk = countForWindow(encoder, DIAGNOSTIC_WINDOW_MS, forwardDelta);
    snprintf(prompt, sizeof(prompt), "DIAGNOSTICO %s (2/2): gire a MESMA roda no sentido oposto por cerca de %lu s; pressione ENTER e comece.",
        name, static_cast<unsigned long>(DIAGNOSTIC_WINDOW_MS / 1000));
    DiagnosticLog::waitForEnter(prompt);
    const bool secondOk = countForWindow(encoder, DIAGNOSTIC_WINDOW_MS, reverseDelta);

    if (!firstOk || !secondOk) {
        DiagnosticLog::log("ERROR", "%s: erro ao ler/resetar PCNT durante diagnostico.", name);
        return;
    }
    DiagnosticLog::log("INFO", "%s resultado: delta sentido A=%ld, delta sentido B=%ld; contagens em pulsos (não equivalem diretamente a RPM sem calibracao mecanica).",
        name, static_cast<long>(forwardDelta), static_cast<long>(reverseDelta));
    if (forwardDelta != 0 && reverseDelta != 0 && ((forwardDelta > 0) != (reverseDelta > 0))) {
        DiagnosticLog::log("PASS", "%s: os dois sentidos produziram contagens de sinais opostos.", name);
    } else {
        DiagnosticLog::log("WARN", "%s: nao foi observada contagem nos dois sentidos com sinal oposto. Repita girando a roda de forma continua e verifique os canais A/B.", name);
    }
}
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.setTimeout(30000);
    DiagnosticLog::banner("Encoders N20 esquerdo/direito (PCNT)");
    DiagnosticLog::log("INFO", "O PCNT conta bordas do canal A; o nivel do canal B determina o sentido.");
    DiagnosticLog::log("WARN", "Os encoders nao possuem protocolo de identificacao: a conexao sera confirmada pela deteccao de pulsos ao girar manualmente as rodas.");

    leftInitialized = encoderLeft.begin(BoardPins::ENCODER_LEFT_A, BoardPins::ENCODER_LEFT_B);
    rightInitialized = encoderRight.begin(BoardPins::ENCODER_RIGHT_A, BoardPins::ENCODER_RIGHT_B);
    DiagnosticLog::log(leftInitialized ? "PASS" : "ERROR", "Inicializacao PCNT esquerdo (GPIO%u/%u): %s.",
        BoardPins::ENCODER_LEFT_A, BoardPins::ENCODER_LEFT_B, leftInitialized ? "OK" : "FALHA");
    DiagnosticLog::log(rightInitialized ? "PASS" : "ERROR", "Inicializacao PCNT direito (GPIO%u/%u): %s.",
        BoardPins::ENCODER_RIGHT_A, BoardPins::ENCODER_RIGHT_B, rightInitialized ? "OK" : "FALHA");

    DiagnosticLog::log("STAGE", "FASE 1 - pre-verificacao de conexao por pulsos antes de qualquer diagnostico direcional.");
    leftConnected = connectionPreflight("ESQUERDO", encoderLeft, leftInitialized, BoardPins::ENCODER_LEFT_A, BoardPins::ENCODER_LEFT_B);
    rightConnected = connectionPreflight("DIREITO", encoderRight, rightInitialized, BoardPins::ENCODER_RIGHT_A, BoardPins::ENCODER_RIGHT_B);

    DiagnosticLog::log("STAGE", "FASE 2 - diagnostico direcional somente para encoders com conexao confirmada.");
    if (leftConnected) diagnoseDirection("ESQUERDO", encoderLeft);
    else DiagnosticLog::log("WARN", "Diagnostico ESQUERDO ignorado porque a conexao nao passou no pre-check.");
    if (rightConnected) diagnoseDirection("DIREITO", encoderRight);
    else DiagnosticLog::log("WARN", "Diagnostico DIREITO ignorado porque a conexao nao passou no pre-check.");
    DiagnosticLog::log("INFO", "Teste de encoders encerrado. Revise cabos, acoplamento mecanico e convencao de sentido antes da odometria.");
}

void loop() {
    delay(1000);
}
