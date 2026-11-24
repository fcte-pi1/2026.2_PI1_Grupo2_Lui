#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "ultrasonic_reading.hpp"

// Nome legado: protocolo Trigger/Echo compatível com HC-SR04 e US-100.
// A montagem foi identificada como HC-SR04; alimentação deve ser conferida.
// Faça as medições dos três sensores
// sequencialmente, com intervalo entre eles, para reduzir diafonia acústica.
class UltrasonicUS100 {
public:
    bool begin(uint8_t triggerPin,
               uint8_t echoPin,
               uint32_t timeoutUs = 25000UL,
               float minDistanceCm = 2.0f,
               float maxDistanceCm = 450.0f);

    // Efetua uma medição bruta. O driver não classifica parede/caminho livre;
    // a validação e a interpretação de ausência de eco pertencem à HU-01.
    UltrasonicReading measure();
    bool setDistanceOffsetCm(float offsetCm);
    float distanceOffsetCm() const { return _distanceOffsetCm; }

private:
    uint8_t _triggerPin = 0;
    uint8_t _echoPin = 0;
    uint32_t _timeoutUs = 25000UL;
    float _minDistanceCm = 2.0f;
    float _distanceOffsetCm = 0.0f;
    float _maxDistanceCm = 450.0f;
    bool _initialized = false;
};
