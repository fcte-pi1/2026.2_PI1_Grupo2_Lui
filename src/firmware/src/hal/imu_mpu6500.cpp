#include "imu_mpu6500.hpp"

#include <math.h>

namespace {
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU6500_WHO_AM_I = 0x70;
constexpr uint8_t MPU6050_COMPAT_WHO_AM_I = 0x68;
constexpr float ACCEL_LSB_PER_G = 16384.0f; // escala +/-2 g
constexpr float GYRO_LSB_PER_DPS = 131.0f;  // escala +/-250 graus/s
}

bool IMUMPU6500::begin(TwoWire& bus, uint8_t address) {
    _bus = &bus;
    _address = address;

    uint8_t id = 0;
    if (!readRegister(REG_WHO_AM_I, id)) {
        return false;
    }
    // 0x70 é o identificador esperado no MPU6500. 0x68 é aceito para
    // módulos compatíveis MPU6050; confira o componente real da placa.
    if (id != MPU6500_WHO_AM_I && id != MPU6050_COMPAT_WHO_AM_I) {
        return false;
    }
    _deviceId = id;

    if (!writeRegister(REG_PWR_MGMT_1, 0x80)) { // reset do dispositivo
        return false;
    }
    delay(100);
    if (!writeRegister(REG_PWR_MGMT_1, 0x01) || // PLL com gyro X
        !writeRegister(REG_CONFIG, 0x03) ||     // DLPF
        !writeRegister(REG_SMPLRT_DIV, 0x04) ||
        !writeRegister(REG_GYRO_CONFIG, 0x00) || // +/-250 dps
        !writeRegister(REG_ACCEL_CONFIG, 0x00) || // +/-2 g
        !writeRegister(REG_ACCEL_CONFIG2, 0x03)) {
        return false;
    }

    _gyroZBiasRaw = 0;
    _yawDegrees = 0.0f;
    _lastUpdateUs = micros();
    _initialized = true;
    return true;
}

bool IMUMPU6500::calibrateGyroZ(uint16_t samples, uint16_t intervalMs) {
    if (!_initialized || samples == 0) {
        return false;
    }

    int64_t sum = 0;
    uint16_t validSamples = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        int16_t rawZ = 0;
        if (readRawGyroZ(rawZ)) {
            sum += rawZ;
            ++validSamples;
        }
        if (intervalMs > 0) {
            delay(intervalMs);
        }
    }

    if (validSamples == 0) {
        return false;
    }

    _gyroZBiasRaw = static_cast<int32_t>(sum / validSamples);
    _lastUpdateUs = micros();
    return true;
}

bool IMUMPU6500::read(MPU6500Reading& reading) {
    if (!_initialized) {
        return false;
    }

    int16_t values[7] = {};
    if (!readFrame(values)) {
        return false;
    }

    const uint32_t nowUs = micros();
    float dtSeconds = static_cast<float>(nowUs - _lastUpdateUs) / 1000000.0f;
    _lastUpdateUs = nowUs;
    if (dtSeconds > 0.25f) {
        // Um intervalo excessivo costuma indicar bloqueio; evita integrar
        // um grande salto temporal de uma única leitura.
        dtSeconds = 0.0f;
    }

    const float gyroX = static_cast<float>(values[4]) / GYRO_LSB_PER_DPS;
    const float gyroY = static_cast<float>(values[5]) / GYRO_LSB_PER_DPS;
    const float gyroZ = (static_cast<float>(values[6]) -
                         static_cast<float>(_gyroZBiasRaw)) /
                        GYRO_LSB_PER_DPS;

    _yawDegrees += gyroZ * dtSeconds;
    while (_yawDegrees >= 180.0f) _yawDegrees -= 360.0f;
    while (_yawDegrees < -180.0f) _yawDegrees += 360.0f;

    reading.accelXg = static_cast<float>(values[0]) / ACCEL_LSB_PER_G;
    reading.accelYg = static_cast<float>(values[1]) / ACCEL_LSB_PER_G;
    reading.accelZg = static_cast<float>(values[2]) / ACCEL_LSB_PER_G;
    reading.gyroXdps = gyroX;
    reading.gyroYdps = gyroY;
    reading.gyroZdps = gyroZ;
    reading.yawDegrees = _yawDegrees;
    reading.timestampUs = nowUs;
    return true;
}

void IMUMPU6500::resetYaw(float yawDegrees) {
    _yawDegrees = yawDegrees;
    _lastUpdateUs = micros();
}

bool IMUMPU6500::isInitialized() const {
    return _initialized;
}

uint8_t IMUMPU6500::deviceId() const {
    return _deviceId;
}

bool IMUMPU6500::writeRegister(uint8_t reg, uint8_t value) {
    if (_bus == nullptr) return false;
    _bus->beginTransmission(_address);
    _bus->write(reg);
    _bus->write(value);
    return _bus->endTransmission() == 0;
}

bool IMUMPU6500::readRegister(uint8_t reg, uint8_t& value) {
    if (_bus == nullptr) return false;
    _bus->beginTransmission(_address);
    _bus->write(reg);
    if (_bus->endTransmission(false) != 0) return false;
    if (_bus->requestFrom(_address, static_cast<uint8_t>(1)) != 1) return false;
    value = static_cast<uint8_t>(_bus->read());
    return true;
}

bool IMUMPU6500::readFrame(int16_t values[7]) {
    if (_bus == nullptr) return false;
    _bus->beginTransmission(_address);
    _bus->write(REG_ACCEL_XOUT_H);
    if (_bus->endTransmission(false) != 0) return false;
    if (_bus->requestFrom(_address, static_cast<uint8_t>(14)) != 14) return false;

    uint8_t bytes[14] = {};
    for (uint8_t i = 0; i < 14; ++i) {
        if (!_bus->available()) return false;
        bytes[i] = static_cast<uint8_t>(_bus->read());
    }

    for (uint8_t i = 0; i < 7; ++i) {
        values[i] = static_cast<int16_t>(
            (static_cast<uint16_t>(bytes[i * 2]) << 8) |
            static_cast<uint16_t>(bytes[i * 2 + 1]));
    }
    return true;
}

bool IMUMPU6500::readRawGyroZ(int16_t& gyroZ) {
    if (_bus == nullptr) return false;
    _bus->beginTransmission(_address);
    _bus->write(REG_GYRO_ZOUT_H);
    if (_bus->endTransmission(false) != 0) return false;
    if (_bus->requestFrom(_address, static_cast<uint8_t>(2)) != 2) return false;
    if (_bus->available() < 2) return false;
    const uint8_t high = static_cast<uint8_t>(_bus->read());
    const uint8_t low = static_cast<uint8_t>(_bus->read());
    gyroZ = static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
    return true;
}

float IMUMPU6500::getYawDegrees() const {
    return _yawDegrees;
}
