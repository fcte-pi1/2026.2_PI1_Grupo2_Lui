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
    static constexpr uint8_t FILTER_WINDOW = 5;

    bool begin(uint8_t triggerPin,
               uint8_t echoPin,
               uint32_t timeoutUs = 25000UL,
               float minDistanceCm = 2.0f,
               float maxDistanceCm = 400.0f);

    // Efetua uma medição. Se o eco falhar, valid=false; não interpreta
    // ausência de eco como corredor livre.
    UltrasonicReading measure();

    void clearFilter();

private:
    float medianOfSamples() const;
    void pushSample(float centimeters);

    uint8_t _triggerPin = 0;
    uint8_t _echoPin = 0;
    uint32_t _timeoutUs = 25000UL;
    float _minDistanceCm = 2.0f;
    float _maxDistanceCm = 400.0f;
    float _samples[FILTER_WINDOW] = {};
    uint8_t _sampleCount = 0;
    uint8_t _nextSample = 0;
    bool _initialized = false;
};
