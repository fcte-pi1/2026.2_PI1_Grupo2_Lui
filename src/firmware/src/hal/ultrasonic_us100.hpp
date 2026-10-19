#pragma once

#include <Arduino.h>
#include <stdint.h>

struct UltrasonicReading {
    bool valid;
    float distanceCm;
    uint32_t pulseUs;
};

// Driver US-100 no modo Trigger/Echo. Faça as medições dos três sensores
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

private:
    uint8_t _triggerPin = 0;
    uint8_t _echoPin = 0;
    uint32_t _timeoutUs = 25000UL;
    float _minDistanceCm = 2.0f;
    float _maxDistanceCm = 450.0f;
    bool _initialized = false;
};
