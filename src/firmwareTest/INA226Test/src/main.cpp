#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "i2c_diagnostic.hpp"
#include "diagnostic_logger.hpp"
#include "../../../firmware/include/config/board_pins.hpp"
#include "../../../firmware/src/hal/ina226.hpp"

namespace {
INA226 monitor;
constexpr float SHUNT_RESISTANCE_OHMS = 0.01f; // EXEMPLO: confirmar hardware real
constexpr float MAX_EXPECTED_CURRENT_A = 5.0f; // EXEMPLO: confirmar hardware real
constexpr unsigned SAMPLE_COUNT = 10;

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
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.setTimeout(30000);
    DiagnosticLog::banner("Medidor INA226");
    DiagnosticLog::log("INFO", "I2C: SDA=GPIO%u, SCL=GPIO%u, clock=%lu Hz; endereco configurado=0x%02X.",
        BoardPins::I2C_SDA, BoardPins::I2C_SCL,
        static_cast<unsigned long>(BoardPins::I2C_CLOCK_HZ), BoardPins::INA226_ADDRESS);
    DiagnosticLog::log("WARN", "Parametros de calibracao deste teste: shunt=%.5f ohm, corrente maxima=%.2f A. Confirme a etiqueta/esquema do modulo antes de confiar nos valores.",
        SHUNT_RESISTANCE_OHMS, MAX_EXPECTED_CURRENT_A);

    if (SHUNT_RESISTANCE_OHMS * MAX_EXPECTED_CURRENT_A > 0.08192f) {
        DiagnosticLog::log("ERROR", "Parametros ultrapassam +/-81.92 mV de faixa do shunt no driver; conexao/calibracao funcional nao sera iniciada.");
        return;
    }
    if (!Wire.begin(BoardPins::I2C_SDA, BoardPins::I2C_SCL)) {
        DiagnosticLog::log("ERROR", "Barramento I2C nao inicializou; diagnostico bloqueado.");
        return;
    }
    Wire.setClock(BoardPins::I2C_CLOCK_HZ);

    DiagnosticLog::log("STAGE", "FASE 1 - teste de conexao: ACK I2C e leitura do registrador Configuration (0x00) antes de configurar/calcular grandezas.");
    uint8_t status = 0xFF;
    if (!DiagnosticI2C::probe(Wire, BoardPins::INA226_ADDRESS, status)) {
        DiagnosticLog::log("ERROR", "INA226 nao respondeu em 0x%02X (status=%u). Diagnostico funcional BLOQUEADO.", BoardPins::INA226_ADDRESS, status);
        DiagnosticLog::log("WARN", "Verifique VCC, GND, SDA/SCL, endereco A0/A1 e barramento compartilhado com MPU6500.");
        return;
    }
    uint16_t originalConfig = 0;
    uint16_t manufacturerId = 0;
    uint16_t dieId = 0;
    if (!readRegister16(BoardPins::INA226_ADDRESS, 0x00, originalConfig) ||
        !readRegister16(BoardPins::INA226_ADDRESS, 0xFE, manufacturerId) ||
        !readRegister16(BoardPins::INA226_ADDRESS, 0xFF, dieId)) {
        DiagnosticLog::log("ERROR", "O endereco ACK, mas falhou a leitura de Configuration/Manufacturer ID/Die ID. Conexao funcional nao confirmada; diagnostico bloqueado.");
        return;
    }
    DiagnosticLog::log("INFO", "Registros de identificacao lidos: Manufacturer ID=0x%04X; Die ID=0x%04X; Configuration=0x%04X.",
        manufacturerId, dieId, originalConfig);
    if (manufacturerId != 0x5449 || (dieId & 0xFFF0U) != 0x2260U) {
        DiagnosticLog::log("ERROR", "O dispositivo em 0x%02X nao apresentou os IDs esperados do INA226 (MFR 0x5449, DIE familia 0x226x). Diagnostico funcional BLOQUEADO para evitar interpretar outro dispositivo como INA226.",
            BoardPins::INA226_ADDRESS);
        DiagnosticLog::log("WARN", "Se o modulo for um clone/variante deliberada, verifique seu datasheet e ajuste conscientemente esta validacao antes de alterar o teste.");
        return;
    }
    DiagnosticLog::log("PASS", "PRE-CHECK COMPLETO: endereco I2C, registradores e identificacao INA226 confirmados em 0x%02X.",
        BoardPins::INA226_ADDRESS);

    DiagnosticLog::log("STAGE", "FASE 2 - configurar calibracao e ler grandezas, somente apos conexao confirmada.");
    if (!monitor.begin(Wire, SHUNT_RESISTANCE_OHMS, MAX_EXPECTED_CURRENT_A, BoardPins::INA226_ADDRESS)) {
        DiagnosticLog::log("ERROR", "Driver INA226 nao conseguiu configurar/calibrar. Verifique shunt/corrente maxima, leitura I2C e modulo.");
        return;
    }
    uint16_t calibrationRegister = 0;
    if (!readRegister16(BoardPins::INA226_ADDRESS, 0x05, calibrationRegister) || calibrationRegister == 0) {
        DiagnosticLog::log("ERROR", "Registrador de calibracao 0x05 nao confirmou valor valido apos a configuracao.");
        return;
    }
    DiagnosticLog::log("PASS", "Driver inicializado; Calibration Register=0x%04X; currentLSB=%.8f A/bit.",
        calibrationRegister, monitor.currentLsbA());
    DiagnosticLog::log("INFO", "Se o monitor estiver em serie com a bateria, mantenha corrente dentro dos limites do modulo e do shunt.");

    unsigned valid = 0;
    float minVoltage = 1000.0f;
    float maxVoltage = -1.0f;
    float voltageSum = 0.0f;
    for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
        INA226Reading reading{};
        if (!monitor.read(reading)) {
            DiagnosticLog::log("ERROR", "Amostra %u/%u: falha de leitura dos registradores de medida.", i + 1, SAMPLE_COUNT);
            delay(300);
            continue;
        }
        if (!isfinite(reading.busVoltageV) || !isfinite(reading.shuntVoltageMv) ||
            !isfinite(reading.currentA) || !isfinite(reading.powerW)) {
            DiagnosticLog::log("ERROR", "Amostra %u/%u: NaN/inf detectado; valores descartados.", i + 1, SAMPLE_COUNT);
            delay(300);
            continue;
        }
        ++valid;
        voltageSum += reading.busVoltageV;
        if (reading.busVoltageV < minVoltage) minVoltage = reading.busVoltageV;
        if (reading.busVoltageV > maxVoltage) maxVoltage = reading.busVoltageV;
        DiagnosticLog::log("INFO", "Amostra %u/%u: Vbus=%.4f V | Vshunt=%+.4f mV | I=%+.4f A | P=%.4f W.",
            i + 1, SAMPLE_COUNT, reading.busVoltageV, reading.shuntVoltageMv,
            reading.currentA, reading.powerW);
        if (fabsf(reading.currentA) > MAX_EXPECTED_CURRENT_A * 1.05f) {
            DiagnosticLog::log("WARN", "Corrente medida excede 105%% do parametro maximo configurado; interrompa carga e revise calibracao/shunt.");
        }
        if (reading.busVoltageV > 10.0f) {
            DiagnosticLog::log("WARN", "Tensao acima de 10 V, superior ao esperado para a bateria 2S deste projeto; confira ponto de medicao e divisao do circuito.");
        }
        delay(300);
    }
    if (valid == SAMPLE_COUNT) {
        DiagnosticLog::log("PASS", "Todas as %u leituras de tensao/corrente/potencia foram lidas sem erro de barramento.", SAMPLE_COUNT);
        DiagnosticLog::log("INFO", "Resumo Vbus: media=%.4f V, minimo=%.4f V, maximo=%.4f V.", voltageSum / valid, minVoltage, maxVoltage);
        if (voltageSum / valid <= 0.05f) {
            DiagnosticLog::log("WARN", "Barramento medido proximo de 0 V: o CI pode estar conectado por I2C, mas a linha monitorada talvez esteja sem alimentacao/ligacao IN+/IN-.");
        }
    } else {
        DiagnosticLog::log("ERROR", "Somente %u/%u leituras validas. Revisar barramento, modulo e cabos de medicao.", valid, SAMPLE_COUNT);
    }
    DiagnosticLog::log("INFO", "Corrente proxima de zero pode ser normal sem carga. Compare sempre com multimetro e com o shunt real; o teste nao certifica calibracao metrologica.");
}

void loop() {
    delay(1000);
}
