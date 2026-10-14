#include "motor_drv8833.hpp"

#include <stdlib.h>

bool MotorDRV8833::begin(uint8_t leftIn1Pin,
                         uint8_t leftIn2Pin,
                         uint8_t rightIn1Pin,
                         uint8_t rightIn2Pin,
                         uint8_t nSleepPin,
                         uint32_t pwmFrequencyHz,
                         uint8_t pwmResolutionBits,
                         bool invertLeftDirection,
                         bool invertRightDirection) {
    if (pwmFrequencyHz == 0 || pwmResolutionBits == 0 ||
        pwmResolutionBits > 12) {
        return false;
    }

    _leftIn1 = leftIn1Pin;
    _leftIn2 = leftIn2Pin;
    _rightIn1 = rightIn1Pin;
    _rightIn2 = rightIn2Pin;
    _nSleepPin = nSleepPin;
    _pwmFrequencyHz = pwmFrequencyHz;
    _pwmResolutionBits = pwmResolutionBits;
    _maxDuty = static_cast<uint16_t>((1U << pwmResolutionBits) - 1U);
    _invertLeft = invertLeftDirection;
    _invertRight = invertRightDirection;

    // Manter a ponte desabilitada durante a configuração inicial.
    pinMode(_nSleepPin, OUTPUT);
    digitalWrite(_nSleepPin, LOW);

    pinMode(_leftIn1, OUTPUT);
    pinMode(_leftIn2, OUTPUT);
    pinMode(_rightIn1, OUTPUT);
    pinMode(_rightIn2, OUTPUT);

    configurePwmPin(_leftIn1);
    configurePwmPin(_leftIn2);
    configurePwmPin(_rightIn1);
    configurePwmPin(_rightIn2);

    stop();
    _initialized = true;
    _enabled = false;
    return true;
}

void MotorDRV8833::configurePwmPin(uint8_t pin) {
    analogWriteResolution(pin, _pwmResolutionBits);
    analogWriteFrequency(pin, _pwmFrequencyHz);
    analogWrite(pin, 0);
}

void MotorDRV8833::setEnabled(bool enabled) {
    if (!_initialized) {
        return;
    }

    if (!enabled) {
        stop();
        digitalWrite(_nSleepPin, LOW);
        _enabled = false;
        return;
    }

    digitalWrite(_nSleepPin, HIGH);
    _enabled = true;
}

bool MotorDRV8833::isEnabled() const {
    return _enabled;
}

void MotorDRV8833::setLeftSpeed(int16_t speed) {
    if (!_initialized || !_enabled) {
        return;
    }
    writeMotor(_leftIn1, _leftIn2, speed, _invertLeft);
}

void MotorDRV8833::setRightSpeed(int16_t speed) {
    if (!_initialized || !_enabled) {
        return;
    }
    writeMotor(_rightIn1, _rightIn2, speed, _invertRight);
}

void MotorDRV8833::setSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
}

void MotorDRV8833::writeMotor(uint8_t in1Pin,
                              uint8_t in2Pin,
                              int16_t speed,
                              bool invertDirection) {
    int32_t command = speed;
    if (command > static_cast<int32_t>(_maxDuty)) {
        command = _maxDuty;
    } else if (command < -static_cast<int32_t>(_maxDuty)) {
        command = -static_cast<int32_t>(_maxDuty);
    }

    if (invertDirection) {
        command = -command;
    }

    if (command > 0) {
        analogWrite(in1Pin, static_cast<uint32_t>(command));
        analogWrite(in2Pin, 0);
    } else if (command < 0) {
        analogWrite(in1Pin, 0);
        analogWrite(in2Pin, static_cast<uint32_t>(-command));
    } else {
        analogWrite(in1Pin, 0);
        analogWrite(in2Pin, 0);
    }
}

void MotorDRV8833::stop() {
    if (!_initialized) {
        // Também é chamado durante begin(), antes de _initialized=true.
        analogWrite(_leftIn1, 0);
        analogWrite(_leftIn2, 0);
        analogWrite(_rightIn1, 0);
        analogWrite(_rightIn2, 0);
        return;
    }
    analogWrite(_leftIn1, 0);
    analogWrite(_leftIn2, 0);
    analogWrite(_rightIn1, 0);
    analogWrite(_rightIn2, 0);
}

void MotorDRV8833::brake() {
    if (!_initialized || !_enabled) {
        return;
    }
    analogWrite(_leftIn1, _maxDuty);
    analogWrite(_leftIn2, _maxDuty);
    analogWrite(_rightIn1, _maxDuty);
    analogWrite(_rightIn2, _maxDuty);
}
