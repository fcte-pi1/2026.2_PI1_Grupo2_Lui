#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include "config/sonar_sampling.hpp"
#include "ultrasonic_reading.hpp"
struct SonarEstimate {
    bool valid=false;
    float distanceCm=NAN;
    uint32_t lastValidAtMs=0;
};
// Window contains the last three acquisition attempts, including failures.
// One missing sample can be covered by two recent real echoes, never forever.
class SonarMedian3 {
public:
    SonarEstimate push(const UltrasonicReading& raw,uint32_t sampledAt) {
        if(hasSample_&&sampledAt==lastSample_)return result_;
        if(hasSample_&&sampledAt-lastSample_>SonarSampling::MAX_SAMPLE_AGE_MS)reset();
        lastSample_=sampledAt;hasSample_=true;
        const bool valid=raw.valid&&!raw.tooClose&&raw.pulseUs&&std::isfinite(raw.distanceCm);
        window_[cursor_]={raw.distanceCm,valid,sampledAt};cursor_=(cursor_+1)%3;
        if(size_<3)++size_;
        float values[3]{};uint8_t count=0;uint32_t latestAge=UINT32_MAX,latest=0;
        for(uint8_t i=0;i<size_;++i) {
            const auto& entry=window_[i];const uint32_t age=sampledAt-entry.timestamp;
            if(!entry.valid||age>SonarSampling::MAX_SAMPLE_AGE_MS)continue;
            values[count++]=entry.cm;
            if(age<latestAge){latestAge=age;latest=entry.timestamp;}
        }
        result_={};
        if(count<2)return result_;
        for(uint8_t i=0;i<count;++i)for(uint8_t j=i+1;j<count;++j)
            if(values[j]<values[i]){const float v=values[i];values[i]=values[j];values[j]=v;}
        result_={true,count==3?values[1]:(values[0]+values[1])*.5f,latest};
        return result_;
    }
    void reset(){*this=SonarMedian3{};}
private:
    struct Entry{float cm=0;bool valid=false;uint32_t timestamp=0;};
    std::array<Entry,3> window_{};
    uint8_t cursor_=0,size_=0;
    uint32_t lastSample_=0;
    bool hasSample_=false;
    SonarEstimate result_{};
};
