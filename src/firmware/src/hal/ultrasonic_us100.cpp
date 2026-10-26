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

    return {true, centimeters, pulseUs};
}
