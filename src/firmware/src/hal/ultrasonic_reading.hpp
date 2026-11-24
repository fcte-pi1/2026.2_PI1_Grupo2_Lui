#pragma once
#include <cstdint>
struct UltrasonicReading {
    bool valid;
    float distanceCm;
    uint32_t pulseUs;
    bool tooClose = false;
};
