#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

struct INA226Reading {
    float busVoltageV;
    float shuntVoltageMv;
    float currentA;
    float powerW;
};

// Driver INA226 sem dependência externa. O valor do shunt real do módulo
// e a corrente máxima esperada são obrigatórios para calibrar corrente/potência.
// Wire deve ter sido inicializado uma única vez pelo chamador.
class INA226 {
public:
    bool begin(TwoWire& bus,
               float shuntResistanceOhms,
               float maxExpectedCurrentA,
               uint8_t address = 0x40);

    bool read(INA226Reading& reading);
    bool isInitialized() const;
    float currentLsbA() const;

private:
    bool writeRegister(uint8_t reg, uint16_t value);
    bool readRegister(uint8_t reg, uint16_t& value);

    TwoWire* _bus = nullptr;
    uint8_t _address = 0x40;
    bool _initialized = false;
    float _currentLsbA = 0.0f;
    float _shuntResistanceOhms = 0.0f;
};
