#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

struct MPU6500Reading {
    float accelXg;
    float accelYg;
    float accelZg;
    float gyroXdps;
    float gyroYdps;
    float gyroZdps;
    float yawDegrees;  // orientação relativa integrada, não norte absoluto
    uint32_t timestampUs;
};

// Driver I2C para MPU6500 compatível. Não inicializa o barramento Wire:
// o chamador deve executar Wire.begin(SDA, SCL) uma única vez para o I2C.
class IMUMPU6500 {
public:
    bool begin(TwoWire& bus = Wire, uint8_t address = 0x68);

    // Calibrar com o robô imóvel, nivelado e com motores desabilitados.
    bool calibrateGyroZ(uint16_t samples = 300, uint16_t intervalMs = 3);

    bool read(MPU6500Reading& reading);
    void resetYaw(float yawDegrees = 0.0f);
    bool isInitialized() const;
    uint8_t deviceId() const;

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    bool readFrame(int16_t values[7]);
    bool readRawGyroZ(int16_t& gyroZ);

    TwoWire* _bus = nullptr;
    uint8_t _address = 0x68;
    uint8_t _deviceId = 0;
    bool _initialized = false;
    int32_t _gyroZBiasRaw = 0;
    float _yawDegrees = 0.0f;
    uint32_t _lastUpdateUs = 0;
};
