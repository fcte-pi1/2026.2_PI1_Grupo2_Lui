#pragma once
#include <cmath>
#include <cstdint>
#include "ultrasonic_reading.hpp"
#include "sonar_median3.hpp"
struct SonarSnapshot {
    UltrasonicReading front{false, NAN, 0};
    UltrasonicReading left{false, NAN, 0};
    UltrasonicReading right{false, NAN, 0};
    uint32_t sampledAtMs = 0; // oldest measurement in this complete frame
    bool ready = false;
    bool hasFilters=false;
    SonarEstimate filteredFront{},filteredLeft{},filteredRight{};
    UltrasonicReading estimate(const UltrasonicReading& raw,const SonarEstimate& filtered,uint32_t now)const {
        if(!hasFilters)return raw;
        const bool valid=filtered.valid&&now-filtered.lastValidAtMs<=SonarSampling::MAX_SAMPLE_AGE_MS;
        // Pulse remains raw; a held estimate does not fabricate an echo pulse.
        return {valid,valid?filtered.distanceCm:NAN,raw.pulseUs,false};
    }
    UltrasonicReading frontEstimate(uint32_t now)const{return estimate(front,filteredFront,now);}
    UltrasonicReading leftEstimate(uint32_t now)const{return estimate(left,filteredLeft,now);}
    UltrasonicReading rightEstimate(uint32_t now)const{return estimate(right,filteredRight,now);}
};
