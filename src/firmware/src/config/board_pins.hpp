#pragma once

#include <stdint.h>

// Pinagem transcrita de docs/4.3 - Projeto conceitual de hardware.md.
// Confirme estes GPIOs com a montagem física e o esquemático vigente.
namespace BoardPins {

struct UltrasonicPins {
    uint8_t trigger;
    uint8_t echo;
};

struct MotorInputPins {
    uint8_t in1;
    uint8_t in2;
};

// US-100 em modo Trigger/Echo (não modo UART).
constexpr UltrasonicPins ULTRASONIC_FRONT = {4, 35};
constexpr UltrasonicPins ULTRASONIC_LEFT  = {14, 36};
constexpr UltrasonicPins ULTRASONIC_RIGHT = {27, 39};

// DRV8833: os dois pinos de cada motor são entradas da ponte H.
// PWM é aplicado a uma entrada e 0 à outra, conforme o sentido.
constexpr MotorInputPins MOTOR_LEFT  = {16, 17};
constexpr MotorInputPins MOTOR_RIGHT = {18, 19};
constexpr uint8_t MOTOR_NSLEEP = 5;  // HIGH habilita o DRV8833
constexpr uint32_t MOTOR_PWM_HZ = 20000;
constexpr uint8_t MOTOR_PWM_BITS = 8;

// Encoders N20: canal A contado por PCNT; canal B define o sentido.
constexpr uint8_t ENCODER_LEFT_A  = 32;
constexpr uint8_t ENCODER_LEFT_B  = 33;
constexpr uint8_t ENCODER_RIGHT_A = 25;
constexpr uint8_t ENCODER_RIGHT_B = 26;

// Barramento I2C compartilhado pelo MPU6500 e INA226.
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint8_t MPU6500_ADDRESS = 0x68; // AD0 baixo; 0x69 se AD0 alto
constexpr uint8_t INA226_ADDRESS = 0x40;  // confirmar A0/A1 no módulo

}  // namespace BoardPins
