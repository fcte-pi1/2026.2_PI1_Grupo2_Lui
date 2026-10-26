#include "ina226.hpp"

#include <math.h>

namespace {
constexpr uint8_t REG_CONFIGURATION = 0x00;
constexpr uint8_t REG_SHUNT_VOLTAGE = 0x01;
constexpr uint8_t REG_BUS_VOLTAGE = 0x02;
constexpr uint8_t REG_POWER = 0x03;
constexpr uint8_t REG_CURRENT = 0x04;
constexpr uint8_t REG_CALIBRATION = 0x05;
constexpr float BUS_VOLTAGE_LSB_V = 0.00125f; // 1.25 mV/bit
constexpr float SHUNT_VOLTAGE_LSB_MV = 0.0025f; // 2.5 uV = 0.0025 mV
constexpr float CALIBRATION_CONSTANT = 0.00512f;
}

bool INA226::begin(TwoWire& bus,
                   float shuntResistanceOhms,
                   float maxExpectedCurrentA,
                   uint8_t address) {
    _bus = &bus;
    _address = address;

    if (!isfinite(shuntResistanceOhms) || !isfinite(maxExpectedCurrentA) ||
        shuntResistanceOhms <= 0.0f || maxExpectedCurrentA <= 0.0f) {
        return false;
    }

    // Faixa do INA226: aproximadamente +/-81.92 mV no shunt.
    if (shuntResistanceOhms * maxExpectedCurrentA > 0.08192f) {
        return false;
    }

    uint16_t deviceConfig = 0;
    if (!readRegister(REG_CONFIGURATION, deviceConfig)) {
        return false; // endereço I2C não respondeu
    }

    // Média 1, conversões de barramento/shunt de 1.1 ms, modo contínuo.
    if (!writeRegister(REG_CONFIGURATION, 0x4127)) {
        return false;
    }

    _currentLsbA = maxExpectedCurrentA / 32768.0f;
    _shuntResistanceOhms = shuntResistanceOhms;
    const float calibrationFloat = CALIBRATION_CONSTANT /
        (_currentLsbA * _shuntResistanceOhms);
    if (!isfinite(calibrationFloat) || calibrationFloat < 1.0f ||
        calibrationFloat > 65535.0f) {
        _currentLsbA = 0.0f;
        return false;
    }

    const uint16_t calibration = static_cast<uint16_t>(calibrationFloat);
    if (!writeRegister(REG_CALIBRATION, calibration)) {
        _currentLsbA = 0.0f;
        return false;
    }

    _initialized = true;
    return true;
}

bool INA226::read(INA226Reading& reading) {
    if (!_initialized) return false;

    uint16_t rawBus = 0;
    uint16_t rawShunt = 0;
    uint16_t rawCurrent = 0;
    uint16_t rawPower = 0;

    if (!readRegister(REG_BUS_VOLTAGE, rawBus) ||
        !readRegister(REG_SHUNT_VOLTAGE, rawShunt) ||
        !readRegister(REG_CURRENT, rawCurrent) ||
        !readRegister(REG_POWER, rawPower)) {
        return false;
    }

    const int16_t signedShunt = static_cast<int16_t>(rawShunt);
    const int16_t signedCurrent = static_cast<int16_t>(rawCurrent);

    reading.busVoltageV = static_cast<float>(rawBus) * BUS_VOLTAGE_LSB_V;
    reading.shuntVoltageMv = static_cast<float>(signedShunt) * SHUNT_VOLTAGE_LSB_MV;
    reading.currentA = static_cast<float>(signedCurrent) * _currentLsbA;
    reading.powerW = static_cast<float>(rawPower) * 25.0f * _currentLsbA;
    return true;
}

bool INA226::isInitialized() const {
    return _initialized;
}

float INA226::currentLsbA() const {
    return _currentLsbA;
}

bool INA226::writeRegister(uint8_t reg, uint16_t value) {
    if (_bus == nullptr) return false;
    _bus->beginTransmission(_address);
    _bus->write(reg);
    _bus->write(static_cast<uint8_t>(value >> 8));
    _bus->write(static_cast<uint8_t>(value & 0xFF));
    return _bus->endTransmission() == 0;
}

bool INA226::readRegister(uint8_t reg, uint16_t& value) {
    if (_bus == nullptr) return false;
    _bus->beginTransmission(_address);
    _bus->write(reg);
    if (_bus->endTransmission(false) != 0) return false;
    if (_bus->requestFrom(_address, static_cast<uint8_t>(2)) != 2) return false;
    if (_bus->available() < 2) return false;

    const uint16_t high = static_cast<uint8_t>(_bus->read());
    const uint16_t low = static_cast<uint8_t>(_bus->read());
    value = static_cast<uint16_t>((high << 8) | low);
    return true;
}
