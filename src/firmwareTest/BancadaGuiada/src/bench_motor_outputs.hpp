#pragma once
#include <Arduino.h>
#include "config/board_pins.hpp"

// Replica o teste LEDC que o usuario confirmou na montagem real.
// Saidas assinadas permitem o ensaio de giro comandado explicitamente pela serial.
class BenchMotorOutputs {
public:
    bool begin() {
        pinMode(BoardPins::MOTOR_NSLEEP, OUTPUT);
        digitalWrite(BoardPins::MOTOR_NSLEEP, LOW);
        pinMode(BoardPins::MOTOR_LEFT.in2, OUTPUT);
        pinMode(BoardPins::MOTOR_RIGHT.in2, OUTPUT);
        digitalWrite(BoardPins::MOTOR_LEFT.in2, LOW);
        digitalWrite(BoardPins::MOTOR_RIGHT.in2, LOW);
        const bool left = ledcAttach(BoardPins::MOTOR_LEFT.in1, 20000, 8);
        const bool right = ledcAttach(BoardPins::MOTOR_RIGHT.in1, 20000, 8);
        leftAttached_ = left;
        rightAttached_ = right;
        setEnabled(false);
        ready_ = left && right;
        return ready_;
    }

    void setEnabled(bool enabled) {
        if (!enabled) {
            if (leftAttached_) ledcWrite(BoardPins::MOTOR_LEFT.in1, 0);
            if (rightAttached_) ledcWrite(BoardPins::MOTOR_RIGHT.in1, 0);
            digitalWrite(BoardPins::MOTOR_LEFT.in2, LOW);
            digitalWrite(BoardPins::MOTOR_RIGHT.in2, LOW);
            digitalWrite(BoardPins::MOTOR_NSLEEP, LOW);
            enabled_ = false;
            return;
        }
        if (!ready_) return;
        digitalWrite(BoardPins::MOTOR_NSLEEP, HIGH);
        delay(10);
        enabled_ = true;
    }

    void setSpeeds(int left, int right) {
        if (!enabled_) return;
        writeSigned(BoardPins::MOTOR_LEFT.in1, BoardPins::MOTOR_LEFT.in2, left);
        writeSigned(BoardPins::MOTOR_RIGHT.in1, BoardPins::MOTOR_RIGHT.in2, right);
    }

    void setTurnOutputs(int left, int right, bool brakeLeft, bool brakeRight) {
        if (!enabled_) return;
        if (brakeLeft) {
            ledcWrite(BoardPins::MOTOR_LEFT.in1, 255);
            digitalWrite(BoardPins::MOTOR_LEFT.in2, HIGH);
        } else writeSigned(BoardPins::MOTOR_LEFT.in1, BoardPins::MOTOR_LEFT.in2, left);
        if (brakeRight) {
            ledcWrite(BoardPins::MOTOR_RIGHT.in1, 255);
            digitalWrite(BoardPins::MOTOR_RIGHT.in2, HIGH);
        } else writeSigned(BoardPins::MOTOR_RIGHT.in1, BoardPins::MOTOR_RIGHT.in2, right);
    }

    void brake() {
        if (!enabled_) return;
        // IN1=IN2=HIGH: frenagem eletrica conforme o driver integrado.
        ledcWrite(BoardPins::MOTOR_LEFT.in1, 255);
        ledcWrite(BoardPins::MOTOR_RIGHT.in1, 255);
        digitalWrite(BoardPins::MOTOR_LEFT.in2, HIGH);
        digitalWrite(BoardPins::MOTOR_RIGHT.in2, HIGH);
    }

private:
    static void writeSigned(uint8_t pwmPin, uint8_t dirPin, int speed) {
        speed = constrain(speed, -255, 255);
        // Mesmo modo PWM+direcao do sketch fornecido pelo usuario.
        // IN1 PWM e IN2 LOW: avanco. IN2 HIGH e duty 255-|speed|: re.
        digitalWrite(dirPin, speed < 0 ? HIGH : LOW);
        ledcWrite(pwmPin, speed < 0 ? 255 + speed : speed);
    }
    bool leftAttached_ = false;
    bool rightAttached_ = false;
    bool ready_ = false;
    bool enabled_ = false;
};
