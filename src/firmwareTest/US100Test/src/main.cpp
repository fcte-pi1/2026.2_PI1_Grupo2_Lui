#include <Arduino.h>
#include <esp_system.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "../../../firmware/src/hal/ultrasonic_us100.hpp"

namespace {

// Pinagem conferida em docs/4.3 - Projeto conceitual de hardware.md e no
// esquemático SCH_Ligacao_dos_componentes_1-P1_2026-09-25.png.
// O nome US100Test é mantido para não quebrar os caminhos já usados no repo,
// mas o hardware-alvo deste teste é o HC-SR04.
struct SensorDiagnostic {
    const char* name;
    uint8_t triggerPin;
    uint8_t echoPin;
    UltrasonicUS100 driver;
    bool initialized = false;
    bool connectionConfirmed = false;
};

SensorDiagnostic sensors[] = {
    {"FRONTAL", 4, 35},  // TRIG GPIO4, ECHO GPIO35
    {"ESQUERDO", 14, 36}, // TRIG GPIO14, ECHO GPIO36 / VP
    {"DIREITO", 27, 39},  // TRIG GPIO27, ECHO GPIO39 / VN
};

constexpr uint32_t ECHO_TIMEOUT_US = 25000UL;
constexpr float MIN_DISTANCE_CM = 2.0f;
constexpr float MAX_DISTANCE_CM = 400.0f; // faixa nominal do HC-SR04
constexpr uint8_t PREFLIGHT_ATTEMPTS = 5;
constexpr uint8_t REQUIRED_VALID_READINGS = 3;
constexpr float STABILITY_TOLERANCE_CM = 5.0f;
constexpr uint8_t DIAGNOSTIC_SAMPLES = 8;
constexpr uint32_t MIN_INTERVAL_BETWEEN_PINGS_MS = 60UL;
constexpr uint32_t BETWEEN_SENSORS_MS = 80UL;

const char* levelName(const char* level) { return level; }

void logLine(const char* level, const char* message) {
    Serial.printf("[%8lu ms] [%-5s] %s\n",
                  static_cast<unsigned long>(millis()),
                  levelName(level), message);
}

void printSystemInfo() {
    Serial.println();
    logLine("INFO", "DIAGNOSTICO DE HARDWARE: HC-SR04 - Trigger/Echo");
    logLine("INFO", "Projeto mantido no diretorio US100Test por compatibilidade.");
    logLine("INFO", "Ordem: pre-verificacao de resposta primeiro; diagnostico funcional depois.");
    Serial.printf("[%8lu ms] [INFO ] ESP32: %s | CPU: %u MHz | heap livre: %u bytes | reset reason: %d\n",
                  static_cast<unsigned long>(millis()),
                  ESP.getChipModel(),
                  static_cast<unsigned int>(ESP.getCpuFreqMHz()),
                  static_cast<unsigned int>(ESP.getFreeHeap()),
                  static_cast<int>(esp_reset_reason()));
    logLine("WARN", "HC-SR04 usa 5 V: cada ECHO DEVE passar por divisor/level shifter antes da ESP32.");
    logLine("INFO", "A pinagem documentada e mantida: frontal 4/35, esquerdo 14/36, direito 27/39.");
}

void waitForEnter() {
    // Descarta CR/LF residual do ENTER anterior antes de aguardar nova entrada.
    while (Serial.available() > 0) {
        (void)Serial.read();
    }
    while (true) {
        if (Serial.available() > 0) {
            const int value = Serial.read();
            if (value == '\r' || value == '\n') {
                return;
            }
        }
        delay(5);
    }
}

void sortAscending(float* values, uint8_t count) {
    for (uint8_t i = 0; i < count; ++i) {
        for (uint8_t j = i + 1; j < count; ++j) {
            if (values[j] < values[i]) {
                const float temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }
}

float medianOf(float* values, uint8_t count) {
    sortAscending(values, count);
    if ((count % 2U) != 0U) {
        return values[count / 2U];
    }
    return (values[(count / 2U) - 1U] + values[count / 2U]) / 2.0f;
}

void printSensorPins(const SensorDiagnostic& sensor) {
    Serial.printf("[%8lu ms] [INFO ] Sensor %s (HC-SR04): TRIG=GPIO%u, ECHO=GPIO%u; faixa=%.0f-%.0f cm; timeout=%lu us.\n",
                  static_cast<unsigned long>(millis()),
                  sensor.name,
                  static_cast<unsigned int>(sensor.triggerPin),
                  static_cast<unsigned int>(sensor.echoPin),
                  MIN_DISTANCE_CM,
                  MAX_DISTANCE_CM,
                  static_cast<unsigned long>(ECHO_TIMEOUT_US));
}

void initializeSensors() {
    logLine("STAGE", "FASE 0 - inicializacao local dos GPIOs; isso NAO comprova conexao fisica.");
    for (SensorDiagnostic& sensor : sensors) {
        sensor.initialized = sensor.driver.begin(
            sensor.triggerPin,
            sensor.echoPin,
            ECHO_TIMEOUT_US,
            MIN_DISTANCE_CM,
            MAX_DISTANCE_CM);

        if (sensor.initialized) {
            char message[120];
            snprintf(message, sizeof(message),
                     "%s: GPIOs configurados (TRIG=%u, ECHO=%u); ainda sem confirmacao do sensor.",
                     sensor.name,
                     static_cast<unsigned int>(sensor.triggerPin),
                     static_cast<unsigned int>(sensor.echoPin));
            logLine("INFO", message);
        } else {
            char message[100];
            snprintf(message, sizeof(message), "%s: falha ao inicializar a configuracao do driver.", sensor.name);
            logLine("ERROR", message);
        }
    }
}

void runConnectionPreflight(SensorDiagnostic& sensor) {
    if (!sensor.initialized) {
        sensor.connectionConfirmed = false;
        logLine("ERROR", "Pre-check cancelado: driver/GPIO nao foi inicializado.");
        return;
    }

    logLine("STAGE", "FASE 1 - pre-verificacao de conexao/resposta do HC-SR04.");
    printSensorPins(sensor);
    Serial.printf("[%8lu ms] [ACTION] Coloque um alvo plano a 10-50 cm EM FRENTE ao sensor %s e pressione ENTER.\n",
                  static_cast<unsigned long>(millis()), sensor.name);
    waitForEnter();

    float validDistances[PREFLIGHT_ATTEMPTS] = {};
    uint8_t validCount = 0;

    for (uint8_t attempt = 0; attempt < PREFLIGHT_ATTEMPTS; ++attempt) {
        const UltrasonicReading reading = sensor.driver.measure();
        if (reading.valid) {
            validDistances[validCount++] = reading.distanceCm;
            Serial.printf("[%8lu ms] [INFO ] %s pre-check %u/%u: eco valido; distancia=%.2f cm, pulso=%lu us.\n",
                          static_cast<unsigned long>(millis()), sensor.name,
                          static_cast<unsigned int>(attempt + 1U),
                          static_cast<unsigned int>(PREFLIGHT_ATTEMPTS),
                          reading.distanceCm,
                          static_cast<unsigned long>(reading.pulseUs));
        } else if (reading.pulseUs > 0) {
            Serial.printf("[%8lu ms] [WARN ] %s pre-check %u/%u: pulso recebido (%lu us), mas distancia fora da faixa %.0f-%.0f cm.\n",
                          static_cast<unsigned long>(millis()), sensor.name,
                          static_cast<unsigned int>(attempt + 1U),
                          static_cast<unsigned int>(PREFLIGHT_ATTEMPTS),
                          static_cast<unsigned long>(reading.pulseUs),
                          MIN_DISTANCE_CM, MAX_DISTANCE_CM);
        } else {
            Serial.printf("[%8lu ms] [WARN ] %s pre-check %u/%u: nenhum pulso ECHO valido dentro do timeout.\n",
                          static_cast<unsigned long>(millis()), sensor.name,
                          static_cast<unsigned int>(attempt + 1U),
                          static_cast<unsigned int>(PREFLIGHT_ATTEMPTS));
        }

        if (attempt + 1U < PREFLIGHT_ATTEMPTS) {
            delay(MIN_INTERVAL_BETWEEN_PINGS_MS);
        }
    }

    if (validCount < REQUIRED_VALID_READINGS) {
        sensor.connectionConfirmed = false;
        Serial.printf("[%8lu ms] [ERROR] %s: resposta operacional NAO CONFIRMADA (%u/%u leituras validas; minimo exigido=%u).\n",
                      static_cast<unsigned long>(millis()), sensor.name,
                      static_cast<unsigned int>(validCount),
                      static_cast<unsigned int>(PREFLIGHT_ATTEMPTS),
                      static_cast<unsigned int>(REQUIRED_VALID_READINGS));
        logLine("ERROR", "Verifique: VCC em 5 V, GND comum, TRIG/ECHO nos GPIOs corretos, divisor no ECHO, alvo e cabos.");
        return;
    }

    // Usa mediana para reduzir o impacto de uma leitura espuria de entrada flutuante.
    float sortedDistances[PREFLIGHT_ATTEMPTS] = {};
    for (uint8_t i = 0; i < validCount; ++i) {
        sortedDistances[i] = validDistances[i];
    }
    const float medianCm = medianOf(sortedDistances, validCount);

    uint8_t consistentCount = 0;
    const float toleranceCm = fmaxf(STABILITY_TOLERANCE_CM, medianCm * 0.15f);
    for (uint8_t i = 0; i < validCount; ++i) {
        if (fabsf(validDistances[i] - medianCm) <= toleranceCm) {
            ++consistentCount;
        }
    }

    Serial.printf("[%8lu ms] [INFO ] %s pre-check: mediana=%.2f cm, tolerancia=%.2f cm, leituras consistentes=%u/%u validas.\n",
                  static_cast<unsigned long>(millis()), sensor.name,
                  medianCm, toleranceCm,
                  static_cast<unsigned int>(consistentCount),
                  static_cast<unsigned int>(validCount));

    sensor.connectionConfirmed = consistentCount >= REQUIRED_VALID_READINGS;
    if (sensor.connectionConfirmed) {
        logLine("PASS", "Resposta operacional do HC-SR04 confirmada por multiplas leituras plausiveis/consistentes.");
        logLine("WARN", "PASS significa resposta compativel com um sensor funcional, nao prova absoluta de conexao fisica.");
    } else {
        logLine("ERROR", "Eco inconsistente; conexao/resposta operacional NAO confirmada. Diagnostico funcional sera bloqueado.");
    }
}

void runFunctionalDiagnostic(SensorDiagnostic& sensor) {
    if (!sensor.connectionConfirmed) {
        Serial.printf("[%8lu ms] [WARN ] Diagnostico funcional de %s IGNORADO: pre-check sem confirmacao.\n",
                      static_cast<unsigned long>(millis()), sensor.name);
        return;
    }

    logLine("STAGE", "FASE 2 - diagnostico funcional; somente sensores aprovados no pre-check entram nesta fase.");
    float distances[DIAGNOSTIC_SAMPLES] = {};
    uint8_t validCount = 0;
    uint8_t invalidCount = 0;
    float minimumCm = MAX_DISTANCE_CM;
    float maximumCm = MIN_DISTANCE_CM;
    float sumCm = 0.0f;

    for (uint8_t sample = 0; sample < DIAGNOSTIC_SAMPLES; ++sample) {
        const UltrasonicReading reading = sensor.driver.measure();
        if (reading.valid) {
            distances[validCount++] = reading.distanceCm;
            sumCm += reading.distanceCm;
            if (reading.distanceCm < minimumCm) minimumCm = reading.distanceCm;
            if (reading.distanceCm > maximumCm) maximumCm = reading.distanceCm;
            Serial.printf("[%8lu ms] [INFO ] %s amostra %u/%u: %.2f cm (%lu us).\n",
                          static_cast<unsigned long>(millis()), sensor.name,
                          static_cast<unsigned int>(sample + 1U),
                          static_cast<unsigned int>(DIAGNOSTIC_SAMPLES),
                          reading.distanceCm,
                          static_cast<unsigned long>(reading.pulseUs));
        } else {
            ++invalidCount;
            Serial.printf("[%8lu ms] [WARN ] %s amostra %u/%u: leitura invalida; pulse=%lu us.\n",
                          static_cast<unsigned long>(millis()), sensor.name,
                          static_cast<unsigned int>(sample + 1U),
                          static_cast<unsigned int>(DIAGNOSTIC_SAMPLES),
                          static_cast<unsigned long>(reading.pulseUs));
        }
        if (sample + 1U < DIAGNOSTIC_SAMPLES) {
            delay(MIN_INTERVAL_BETWEEN_PINGS_MS);
        }
    }

    Serial.printf("[%8lu ms] [STAGE] Resultado %s: validas=%u/%u, invalidas=%u/%u.\n",
                  static_cast<unsigned long>(millis()), sensor.name,
                  static_cast<unsigned int>(validCount),
                  static_cast<unsigned int>(DIAGNOSTIC_SAMPLES),
                  static_cast<unsigned int>(invalidCount),
                  static_cast<unsigned int>(DIAGNOSTIC_SAMPLES));

    if (validCount > 0) {
        Serial.printf("[%8lu ms] [INFO ] %s: media=%.2f cm, minimo=%.2f cm, maximo=%.2f cm.\n",
                      static_cast<unsigned long>(millis()), sensor.name,
                      sumCm / validCount, minimumCm, maximumCm);
    }
    if (validCount == DIAGNOSTIC_SAMPLES) {
        logLine("PASS", "Todas as amostras funcionais foram validas.");
    } else {
        logLine("WARN", "Diagnostico concluido com leituras ausentes/invalidas; confira alvo, alimentacao e cabos.");
    }
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(800);
    printSystemInfo();
    logLine("INFO", "Amostragem sequencial; intervalo minimo de 60 ms entre disparos para reduzir diafonia.");

    initializeSensors();

    logLine("STAGE", "FASE 1 - pre-verificacao individual dos sensores antes de qualquer estatistica funcional.");
    for (SensorDiagnostic& sensor : sensors) {
        runConnectionPreflight(sensor);
        delay(BETWEEN_SENSORS_MS);
    }

    logLine("STAGE", "FASE 2 - diagnostico funcional apenas dos sensores com resposta operacional confirmada.");
    uint8_t confirmedCount = 0;
    for (SensorDiagnostic& sensor : sensors) {
        if (sensor.connectionConfirmed) {
            ++confirmedCount;
        }
        runFunctionalDiagnostic(sensor);
        delay(BETWEEN_SENSORS_MS);
    }

    Serial.printf("[%8lu ms] [INFO ] FIM DO TESTE: respostas operacionais confirmadas=%u/%u sensores.\n",
                  static_cast<unsigned long>(millis()),
                  static_cast<unsigned int>(confirmedCount),
                  static_cast<unsigned int>(sizeof(sensors) / sizeof(sensors[0])));
    logLine("INFO", "Para repetir, pressione RESET na ESP32 ou reinicie o monitor/placa.");
}

void loop() {
    delay(1000);
}
