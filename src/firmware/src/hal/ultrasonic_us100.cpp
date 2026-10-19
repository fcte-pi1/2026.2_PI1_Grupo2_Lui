#include "ultrasonic_us100.hpp"

#include <math.h>

bool UltrasonicUS100::begin(uint8_t triggerPin,
                            uint8_t echoPin,
                            uint32_t timeoutUs,
                            float minDistanceCm,
                            float maxDistanceCm) {
    if (timeoutUs == 0 || minDistanceCm <= 0.0f ||
        maxDistanceCm <= minDistanceCm) {
        return false;
    }

    _triggerPin = triggerPin;
    _echoPin = echoPin;
    _timeoutUs = timeoutUs;
    _minDistanceCm = minDistanceCm;
    _maxDistanceCm = maxDistanceCm;

    pinMode(_triggerPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    digitalWrite(_triggerPin, LOW);
    clearFilter();
    _initialized = true;
    return true;
}

UltrasonicReading UltrasonicUS100::measure() {
    if (!_initialized) {
        return {false, NAN, 0};
    }

    // Pulso de disparo compatível com US-100 em modo Trigger/Echo.
    digitalWrite(_triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_triggerPin, LOW);

    const uint32_t pulseUs = pulseIn(_echoPin, HIGH, _timeoutUs);
    if (pulseUs == 0) {
        return {false, NAN, 0};
    }

    // Aproximação usual para ultrassom: duração do eco / 58 = centímetros.
    const float centimeters = static_cast<float>(pulseUs) / 58.0f;
    if (!isfinite(centimeters) || centimeters < _minDistanceCm ||
        centimeters > _maxDistanceCm) {
        return {false, NAN, pulseUs};
    }

    pushSample(centimeters);
    return {true, medianOfSamples(), pulseUs};
}

void UltrasonicUS100::clearFilter() {
    _sampleCount = 0;
    _nextSample = 0;
    for (uint8_t i = 0; i < FILTER_WINDOW; ++i) {
        _samples[i] = 0.0f;
    }
}

void UltrasonicUS100::pushSample(float centimeters) {
    _samples[_nextSample] = centimeters;
    _nextSample = static_cast<uint8_t>((_nextSample + 1) % FILTER_WINDOW);
    if (_sampleCount < FILTER_WINDOW) {
        ++_sampleCount;
    }
}

float UltrasonicUS100::medianOfSamples() const {
    float sorted[FILTER_WINDOW] = {};
    for (uint8_t i = 0; i < _sampleCount; ++i) {
        sorted[i] = _samples[i];
    }

    for (uint8_t i = 1; i < _sampleCount; ++i) {
        const float value = sorted[i];
        int j = static_cast<int>(i) - 1;
        while (j >= 0 && sorted[j] > value) {
            sorted[j + 1] = sorted[j];
            --j;
        }
        sorted[j + 1] = value;
    }

    if (_sampleCount == 0) {
        return NAN;
    }
    if ((_sampleCount % 2U) == 1U) {
        return sorted[_sampleCount / 2U];
    }
    const uint8_t upper = _sampleCount / 2U;
    return (sorted[upper - 1U] + sorted[upper]) / 2.0f;
}
