#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

#include "i2c_diagnostic.hpp"
#include "diagnostic_logger.hpp"
#include "../../../firmware/include/config/board_pins.hpp"
#include "../../../firmware/src/hal/ina226.hpp"

// Valores padrao conforme a secao "Medidor INA226" de docs/7.3.
// Sobrescreva por build_flags se a resistencia REAL do modulo for diferente.
#ifndef INA226_TEST_SHUNT_RESISTANCE_OHMS
#define INA226_TEST_SHUNT_RESISTANCE_OHMS 0.1f
#endif
#ifndef INA226_TEST_MAX_EXPECTED_CURRENT_A
#define INA226_TEST_MAX_EXPECTED_CURRENT_A 0.8f
#endif

namespace {
INA226 monitor;

constexpr uint32_t TEST_I2C_CLOCK_HZ = 400000UL;
constexpr float SHUNT_RESISTANCE_OHMS = INA226_TEST_SHUNT_RESISTANCE_OHMS;
constexpr float MAX_EXPECTED_CURRENT_A = INA226_TEST_MAX_EXPECTED_CURRENT_A;
constexpr float INA226_SHUNT_RANGE_V = 0.08192f;  // +/-81,92 mV
constexpr float VOLTAGE_POINT_ERROR_LIMIT_PERCENT = 0.27f;  // matriz de testes, ID 3.8
constexpr float VOLTAGE_MEAN_ERROR_LIMIT_PERCENT = 0.25f;   // relatorio deste ensaio
constexpr float CURRENT_POINT_ERROR_LIMIT_PERCENT = 0.27f;  // matriz de testes, ID 3.8
constexpr unsigned SAMPLE_COUNT = 10;
constexpr unsigned SAMPLE_INTERVAL_MS = 250;

struct LoadCondition {
    const char* name;
    const char* operatorInstruction;
    float documentedMultimeterVoltageV;
    float documentedInaVoltageV;
    float documentedMultimeterCurrentmA;
    float documentedInaCurrentmA;
};

// Valores de referencia publicados no relatorio. Eles sao informativos; a
// aprovacao compara as leituras desta execucao com valores reais inseridos pelo operador.
const LoadCondition LOAD_CONDITIONS[] = {
    {"Repouso (Standby)",
     "Mantenha os motores desligados e o sistema em repouso.",
     7.82f, 7.81f, 62.0f, 61.8f},
    {"Motores livres (50% PWM)",
     "Com o robo seguro e as rodas livres, aplique 50% PWM aos motores.",
     7.65f, 7.64f, 185.0f, 184.2f},
    {"Motores livres (100% PWM)",
     "Com o robo seguro e as rodas livres, aplique 100% PWM aos motores.",
     7.48f, 7.47f, 320.0f, 319.1f},
    {"Carga maxima / manobra",
     "Reproduza a carga maxima prevista sem travar os motores; interrompa se algo aquecer ou fizer ruido anormal.",
     7.20f, 7.18f, 850.0f, 847.5f},
};
constexpr unsigned LOAD_CONDITION_COUNT = sizeof(LOAD_CONDITIONS) / sizeof(LOAD_CONDITIONS[0]);

struct StageResult {
    bool started = false;
    bool skipped = false;
    bool samplesComplete = false;
    bool multimeterEntered = false;
    bool currentComparable = false;
    bool shuntRangeExceeded = false;
    bool voltagePassed = false;
    bool currentPassed = false;
    unsigned validSamples = 0;
    float averageBusVoltageV = 0.0f;
    float averageShuntVoltageMv = 0.0f;
    float averageCurrentA = 0.0f;
    float averagePowerW = 0.0f;
    float minBusVoltageV = 0.0f;
    float maxBusVoltageV = 0.0f;
    float multimeterVoltageV = 0.0f;
    float multimeterCurrentmA = 0.0f;
    float voltageErrorPercent = 0.0f;
    float currentErrorPercent = 0.0f;
};

bool readRegister16(uint8_t address, uint8_t reg, uint16_t& value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    const uint8_t received = Wire.requestFrom(address, static_cast<uint8_t>(2));
    if (received != 2 || Wire.available() < 2) return false;

    const uint16_t high = static_cast<uint8_t>(Wire.read());
    const uint16_t low = static_cast<uint8_t>(Wire.read());
    value = static_cast<uint16_t>((high << 8) | low);
    return true;
}

bool parsePositiveFloat(String input, float& value) {
    input.trim();
    input.replace(',', '.');  // aceita tanto 7.82 quanto 7,82 no monitor serial
    if (input.length() == 0) return false;

    char* end = nullptr;
    const char* start = input.c_str();
    const float parsed = strtof(start, &end);
    if (end == start || !isfinite(parsed) || parsed <= 0.0f) return false;
    while (*end != '\0' && isspace(static_cast<unsigned char>(*end))) ++end;
    if (*end != '\0') return false;

    value = parsed;
    return true;
}

bool readPositiveFloat(const char* prompt, float& value) {
    for (;;) {
        DiagnosticLog::log("ACTION", "%s", prompt);
        const String input = DiagnosticLog::readLine();
        if (parsePositiveFloat(input, value)) return true;
        DiagnosticLog::log("WARN", "Valor invalido. Digite um numero maior que zero; ponto ou virgula decimal sao aceitos.");
    }
}

bool waitForStageStart(const LoadCondition& condition) {
    DiagnosticLog::log("STAGE", "Condicao: %s", condition.name);
    DiagnosticLog::log("INFO", "Referencia publicada: multimetro %.3f V / %.1f mA; INA226 %.3f V / %.1f mA.",
        condition.documentedMultimeterVoltageV, condition.documentedMultimeterCurrentmA,
        condition.documentedInaVoltageV, condition.documentedInaCurrentmA);
    DiagnosticLog::log("ACTION", "%s", condition.operatorInstruction);
    DiagnosticLog::log("ACTION", "Ajuste a carga, aguarde estabilizar e pressione ENTER para medir; digite 's' para pular esta etapa.");
    const String answer = DiagnosticLog::readLine();
    return !answer.equalsIgnoreCase("s") && !answer.equalsIgnoreCase("pular");
}

bool collectSamples(StageResult& result) {
    float voltageSum = 0.0f;
    float shuntSum = 0.0f;
    float currentSum = 0.0f;
    float powerSum = 0.0f;
    result.minBusVoltageV = 1000.0f;
    result.maxBusVoltageV = -1000.0f;

    for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
        INA226Reading reading{};
        if (!monitor.read(reading)) {
            DiagnosticLog::log("ERROR", "Amostra %u/%u: falha ao ler os registradores do INA226.", i + 1, SAMPLE_COUNT);
            delay(SAMPLE_INTERVAL_MS);
            continue;
        }
        if (!isfinite(reading.busVoltageV) || !isfinite(reading.shuntVoltageMv) ||
            !isfinite(reading.currentA) || !isfinite(reading.powerW)) {
            DiagnosticLog::log("ERROR", "Amostra %u/%u descartada: leitura NaN/inf.", i + 1, SAMPLE_COUNT);
            delay(SAMPLE_INTERVAL_MS);
            continue;
        }

        ++result.validSamples;
        voltageSum += reading.busVoltageV;
        shuntSum += reading.shuntVoltageMv;
        currentSum += reading.currentA;
        powerSum += reading.powerW;
        if (reading.busVoltageV < result.minBusVoltageV) result.minBusVoltageV = reading.busVoltageV;
        if (reading.busVoltageV > result.maxBusVoltageV) result.maxBusVoltageV = reading.busVoltageV;
        if (fabsf(reading.shuntVoltageMv) >= 80.0f) result.shuntRangeExceeded = true;

        DiagnosticLog::log("INFO", "Amostra %u/%u: Vbus=%.4f V | Vshunt=%+.4f mV | I=%+.3f mA | P=%.4f W.",
            i + 1, SAMPLE_COUNT, reading.busVoltageV, reading.shuntVoltageMv,
            reading.currentA * 1000.0f, reading.powerW);
        if (fabsf(reading.currentA) > MAX_EXPECTED_CURRENT_A * 1.05f) {
            DiagnosticLog::log("WARN", "Corrente medida acima de 105%% do limite configurado (%.3f A). Revise a configuracao e a carga.",
                MAX_EXPECTED_CURRENT_A);
        }
        if (reading.busVoltageV > 10.0f) {
            DiagnosticLog::log("WARN", "Tensao acima dos 10 V esperados para a bateria 2S; confira o ponto de medicao.");
        }
        delay(SAMPLE_INTERVAL_MS);
    }

    result.samplesComplete = result.validSamples == SAMPLE_COUNT;
    if (result.validSamples == 0) {
        DiagnosticLog::log("ERROR", "Nenhuma amostra valida nesta condicao.");
        return false;
    }

    const float divisor = static_cast<float>(result.validSamples);
    result.averageBusVoltageV = voltageSum / divisor;
    result.averageShuntVoltageMv = shuntSum / divisor;
    result.averageCurrentA = currentSum / divisor;
    result.averagePowerW = powerSum / divisor;
    DiagnosticLog::log("INFO", "Media (%u/%u amostras): Vbus=%.4f V | Vshunt=%+.4f mV | I=%+.3f mA | P=%.4f W.",
        result.validSamples, SAMPLE_COUNT, result.averageBusVoltageV,
        result.averageShuntVoltageMv, result.averageCurrentA * 1000.0f,
        result.averagePowerW);
    DiagnosticLog::log("INFO", "Variacao de Vbus: minimo=%.4f V; maximo=%.4f V.",
        result.minBusVoltageV, result.maxBusVoltageV);

    if (!result.samplesComplete) {
        DiagnosticLog::log("ERROR", "Etapa incompleta: sao necessarias %u/%u amostras validas para aprovacao.", SAMPLE_COUNT, SAMPLE_COUNT);
    }
    if (result.shuntRangeExceeded) {
        DiagnosticLog::log("WARN", "Vshunt atingiu 80 mV ou mais, proximo do limite de +/-81,92 mV. A comparacao de corrente desta etapa sera inconclusiva.");
    }
    return result.samplesComplete;
}

void compareWithMultimeter(const LoadCondition& condition, StageResult& result) {
    char prompt[150];
    snprintf(prompt, sizeof(prompt), "Informe a tensao medida AGORA no multimetro (V) para '%s':", condition.name);
    readPositiveFloat(prompt, result.multimeterVoltageV);
    snprintf(prompt, sizeof(prompt), "Informe a corrente medida AGORA no multimetro (mA) para '%s':", condition.name);
    readPositiveFloat(prompt, result.multimeterCurrentmA);
    result.multimeterEntered = true;

    result.voltageErrorPercent =
        ((result.averageBusVoltageV - result.multimeterVoltageV) / result.multimeterVoltageV) * 100.0f;
    result.voltagePassed = result.samplesComplete &&
        fabsf(result.voltageErrorPercent) < VOLTAGE_POINT_ERROR_LIMIT_PERCENT;

    const float referenceCurrentA = result.multimeterCurrentmA / 1000.0f;
    const bool configuredRangeSupportsReference =
        referenceCurrentA <= MAX_EXPECTED_CURRENT_A &&
        (SHUNT_RESISTANCE_OHMS * referenceCurrentA) < INA226_SHUNT_RANGE_V;
    result.currentComparable = result.samplesComplete && !result.shuntRangeExceeded &&
        configuredRangeSupportsReference;
    result.currentErrorPercent =
        (((result.averageCurrentA * 1000.0f) - result.multimeterCurrentmA) /
          result.multimeterCurrentmA) * 100.0f;
    result.currentPassed = result.currentComparable &&
        fabsf(result.currentErrorPercent) < CURRENT_POINT_ERROR_LIMIT_PERCENT;

    DiagnosticLog::log("RESULT", "Erro Vbus = %+.4f%% (limite por ponto: < %.2f%%): %s.",
        result.voltageErrorPercent, VOLTAGE_POINT_ERROR_LIMIT_PERCENT,
        result.voltagePassed ? "APROVADO" : "REPROVADO");
    if (result.currentComparable) {
        DiagnosticLog::log("RESULT", "Erro corrente = %+.4f%% (limite por ponto: < %.2f%%): %s.",
            result.currentErrorPercent, CURRENT_POINT_ERROR_LIMIT_PERCENT,
            result.currentPassed ? "APROVADO" : "REPROVADO");
    } else {
        DiagnosticLog::log("WARN", "Corrente: COMPARACAO INCONCLUSIVA. Multimetro=%.1f mA, maximo configurado=%.1f mA, shunt=%.4f ohm; confira a faixa do shunt e a configuracao real.",
            result.multimeterCurrentmA, MAX_EXPECTED_CURRENT_A * 1000.0f,
            SHUNT_RESISTANCE_OHMS);
        DiagnosticLog::log("INFO", "Erro numerico de corrente observado (nao usado para aprovacao): %+.4f%%.", result.currentErrorPercent);
    }
}

void printSummary(const StageResult results[LOAD_CONDITION_COUNT]) {
    unsigned completedStages = 0;
    unsigned voltageComparisons = 0;
    unsigned currentComparisons = 0;
    unsigned voltageFailures = 0;
    unsigned currentFailures = 0;
    unsigned inconclusiveCurrents = 0;
    float voltageAbsoluteErrorSum = 0.0f;

    Serial.println();
    Serial.println("================ RESUMO INA226 ================");
    for (unsigned i = 0; i < LOAD_CONDITION_COUNT; ++i) {
        const StageResult& result = results[i];
        if (result.skipped || !result.started) {
            DiagnosticLog::log("SUMMARY", "%s: PULADA.", LOAD_CONDITIONS[i].name);
            continue;
        }
        if (!result.samplesComplete || !result.multimeterEntered) {
            DiagnosticLog::log("SUMMARY", "%s: INCOMPLETA.", LOAD_CONDITIONS[i].name);
            continue;
        }

        ++completedStages;
        ++voltageComparisons;
        voltageAbsoluteErrorSum += fabsf(result.voltageErrorPercent);
        if (!result.voltagePassed) ++voltageFailures;
        if (result.currentComparable) {
            ++currentComparisons;
            if (!result.currentPassed) ++currentFailures;
        } else {
            ++inconclusiveCurrents;
        }

        DiagnosticLog::log("SUMMARY", "%s: Vbus %.4f V, I %.3f mA, erro V %+.4f%%, erro I %+.4f%% (%s).",
            LOAD_CONDITIONS[i].name, result.averageBusVoltageV,
            result.averageCurrentA * 1000.0f, result.voltageErrorPercent,
            result.currentErrorPercent,
            result.currentComparable ? (result.currentPassed ? "corrente OK" : "corrente FORA DO LIMITE") : "corrente inconclusiva");
    }

    if (voltageComparisons > 0) {
        const float meanAbsoluteVoltageError = voltageAbsoluteErrorSum / voltageComparisons;
        DiagnosticLog::log("SUMMARY", "Erro relativo absoluto medio de Vbus: %.4f%% (criterio do relatorio: < %.2f%%).",
            meanAbsoluteVoltageError, VOLTAGE_MEAN_ERROR_LIMIT_PERCENT);
        if (meanAbsoluteVoltageError >= VOLTAGE_MEAN_ERROR_LIMIT_PERCENT) ++voltageFailures;
    } else {
        DiagnosticLog::log("WARN", "Nao ha comparacoes validas de tensao.");
    }

    if (voltageFailures > 0 || currentFailures > 0) {
        DiagnosticLog::log("ERROR", "RESULTADO FINAL: REPROVADO. Revise erros por condicao e os limites especificados.");
    } else if (completedStages < LOAD_CONDITION_COUNT || voltageComparisons < LOAD_CONDITION_COUNT ||
               inconclusiveCurrents > 0 || currentComparisons < LOAD_CONDITION_COUNT) {
        DiagnosticLog::log("WARN", "RESULTADO FINAL: INCONCLUSIVO. Ha etapas puladas/incompletas ou comparacoes de corrente fora da faixa calibrada.");
    } else {
        DiagnosticLog::log("PASS", "RESULTADO FINAL: APROVADO em todas as quatro condicoes de carga.");
    }

    DiagnosticLog::log("INFO", "Etapas completas: %u/%u | comparacoes V: %u | comparacoes I validas: %u | I inconclusivas: %u.",
        completedStages, LOAD_CONDITION_COUNT, voltageComparisons,
        currentComparisons, inconclusiveCurrents);
    Serial.println("================================================");
}
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.setTimeout(60000);
    DiagnosticLog::banner("Medidor INA226 - tensao/corrente sob carga");

    DiagnosticLog::log("INFO", "I2C Fast-Mode: SDA=GPIO%u, SCL=GPIO%u, clock=%lu Hz, endereco=0x%02X.",
        BoardPins::I2C_SDA, BoardPins::I2C_SCL,
        static_cast<unsigned long>(TEST_I2C_CLOCK_HZ), BoardPins::INA226_ADDRESS);
    DiagnosticLog::log("INFO", "Alimentacao logica indicada na secao 7.3: 3,3 V; shunt configurado=%.4f ohm; corrente maxima de calibracao=%.3f A.",
        SHUNT_RESISTANCE_OHMS, MAX_EXPECTED_CURRENT_A);
    DiagnosticLog::log("INFO", "O teste fara 10 amostras em repouso, 50%% PWM, 100%% PWM e carga maxima; comparara medias com leituras reais do multimetro.");
    DiagnosticLog::log("INFO", "Criterios: erro por ponto de tensao/corrente < %.2f%%; erro absoluto medio de tensao < %.2f%%.",
        VOLTAGE_POINT_ERROR_LIMIT_PERCENT, VOLTAGE_MEAN_ERROR_LIMIT_PERCENT);

    if (!isfinite(SHUNT_RESISTANCE_OHMS) || !isfinite(MAX_EXPECTED_CURRENT_A) ||
        SHUNT_RESISTANCE_OHMS <= 0.0f || MAX_EXPECTED_CURRENT_A <= 0.0f ||
        SHUNT_RESISTANCE_OHMS * MAX_EXPECTED_CURRENT_A > INA226_SHUNT_RANGE_V) {
        DiagnosticLog::log("ERROR", "Configuracao invalida: Rshunt x Imax deve ser <= 81,92 mV. Ajuste o shunt e a corrente conforme o modulo REAL; teste funcional bloqueado.");
        return;
    }

    if (SHUNT_RESISTANCE_OHMS * 0.850f > INA226_SHUNT_RANGE_V) {
        DiagnosticLog::log("WARN", "INCOMPATIBILIDADE DA ESPECIFICACAO: shunt %.4f ohm a 850 mA produz %.2f mV, acima da faixa de +/-81,92 mV do INA226.",
            SHUNT_RESISTANCE_OHMS, SHUNT_RESISTANCE_OHMS * 0.850f * 1000.0f);
        DiagnosticLog::log("WARN", "Com a configuracao atual, a corrente da etapa 'Carga maxima / manobra' nao sera aprovada se exceder a faixa calibrada. Confirme o shunt real antes de testar essa carga.");
    }

    if (!Wire.begin(BoardPins::I2C_SDA, BoardPins::I2C_SCL)) {
        DiagnosticLog::log("ERROR", "Falha ao inicializar o barramento I2C. Teste bloqueado.");
        return;
    }
    Wire.setClock(TEST_I2C_CLOCK_HZ);

    DiagnosticLog::log("STAGE", "FASE 1 - verificar comunicacao I2C e identidade do dispositivo antes de ler grandezas.");
    uint8_t status = 0xFF;
    if (!DiagnosticI2C::probe(Wire, BoardPins::INA226_ADDRESS, status)) {
        DiagnosticLog::log("ERROR", "INA226 nao respondeu em 0x%02X (status I2C=%u). Verifique VCC, GND, SDA/SCL, pull-ups e A0/A1.",
            BoardPins::INA226_ADDRESS, status);
        return;
    }

    uint16_t originalConfig = 0;
    uint16_t manufacturerId = 0;
    uint16_t dieId = 0;
    if (!readRegister16(BoardPins::INA226_ADDRESS, 0x00, originalConfig) ||
        !readRegister16(BoardPins::INA226_ADDRESS, 0xFE, manufacturerId) ||
        !readRegister16(BoardPins::INA226_ADDRESS, 0xFF, dieId)) {
        DiagnosticLog::log("ERROR", "O endereco respondeu, mas falhou a leitura de Configuration/Manufacturer ID/Die ID. Teste funcional bloqueado.");
        return;
    }
    DiagnosticLog::log("INFO", "Manufacturer ID=0x%04X; Die ID=0x%04X; Configuration original=0x%04X.",
        manufacturerId, dieId, originalConfig);
    if (manufacturerId != 0x5449 || (dieId & 0xFFF0U) != 0x2260U) {
        DiagnosticLog::log("ERROR", "IDs esperados do INA226 nao confirmados (MFR 0x5449, DIE familia 0x226x). Teste funcional bloqueado.");
        return;
    }
    DiagnosticLog::log("PASS", "PRE-CHECK: endereco I2C 0x%02X e identidade INA226 confirmados.", BoardPins::INA226_ADDRESS);

    DiagnosticLog::log("STAGE", "FASE 2 - configurar o driver usado pelo firmware principal e validar o registrador de calibracao.");
    if (!monitor.begin(Wire, SHUNT_RESISTANCE_OHMS, MAX_EXPECTED_CURRENT_A, BoardPins::INA226_ADDRESS)) {
        DiagnosticLog::log("ERROR", "O driver nao conseguiu configurar/calibrar o INA226. Verifique o shunt fisico e os parametros.");
        return;
    }
    uint16_t calibrationRegister = 0;
    if (!readRegister16(BoardPins::INA226_ADDRESS, 0x05, calibrationRegister) || calibrationRegister == 0) {
        DiagnosticLog::log("ERROR", "O registrador de calibracao 0x05 nao confirmou um valor valido. Teste bloqueado.");
        return;
    }
    DiagnosticLog::log("PASS", "Driver inicializado; Calibration Register=0x%04X; currentLSB=%.8f A/bit.",
        calibrationRegister, monitor.currentLsbA());

    DiagnosticLog::log("WARN", "Use leituras atuais do multimetro para a comparacao. Os numeros impressos como referencia sao apenas os valores publicados no documento, nao substituem a medicao desta execucao.");
    DiagnosticLog::log("WARN", "Durante PWM, mantenha o robo firmemente apoiado, rodas livres e cabos longe das partes moveis. Nunca conecte o multimetro em modo corrente em paralelo com a bateria.");

    StageResult results[LOAD_CONDITION_COUNT];
    for (unsigned i = 0; i < LOAD_CONDITION_COUNT; ++i) {
        StageResult& result = results[i];
        const LoadCondition& condition = LOAD_CONDITIONS[i];
        if (!waitForStageStart(condition)) {
            result.skipped = true;
            DiagnosticLog::log("WARN", "Etapa '%s' pulada pelo operador.", condition.name);
            continue;
        }

        result.started = true;
        if (!collectSamples(result)) {
            DiagnosticLog::log("ERROR", "A etapa '%s' nao obteve todas as amostras necessarias.", condition.name);
            continue;
        }
        compareWithMultimeter(condition, result);
    }

    printSummary(results);
}

void loop() {
    delay(1000);
}
