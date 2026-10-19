#pragma once

#include <Arduino.h>
#include <stdint.h>

// Driver para DRV8833 em modo IN/IN. speed vai de -255 a +255; o sinal
// determina a direção elétrica e a montagem física determina o sentido real.
class MotorDRV8833 {
public:
    bool begin(uint8_t leftIn1Pin,
               uint8_t leftIn2Pin,
               uint8_t rightIn1Pin,
               uint8_t rightIn2Pin,
               uint8_t nSleepPin,
               uint32_t pwmFrequencyHz = 20000UL,
               uint8_t pwmResolutionBits = 8,
               bool invertLeftDirection = false,
               bool invertRightDirection = false);

    // Por segurança, begin deixa nSLEEP em LOW. Habilite deliberadamente.
    void setEnabled(bool enabled);
    bool isEnabled() const;

    void setLeftSpeed(int16_t speed);
    void setRightSpeed(int16_t speed);
    void setSpeeds(int16_t leftSpeed, int16_t rightSpeed);

    // stop() deixa ambas as entradas em LOW (coast). brake() leva ambas
    // as entradas a HIGH (brake elétrico), quando o driver está habilitado.
    void stop();
    void brake();

private:
    void configurePwmPin(uint8_t pin);
    void writeMotor(uint8_t in1Pin, uint8_t in2Pin,
                    int16_t speed, bool invertDirection);

    uint8_t _leftIn1 = 0;
    uint8_t _leftIn2 = 0;
    uint8_t _rightIn1 = 0;
    uint8_t _rightIn2 = 0;
    uint8_t _nSleepPin = 0;
    uint32_t _pwmFrequencyHz = 20000UL;
    uint8_t _pwmResolutionBits = 8;
    uint16_t _maxDuty = 255;
    bool _invertLeft = false;
    bool _invertRight = false;
    bool _initialized = false;
    bool _enabled = false;
};
