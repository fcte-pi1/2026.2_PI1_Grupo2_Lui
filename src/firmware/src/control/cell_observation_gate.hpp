#pragma once
#include <cstdint>
#include "../config/sonar_sampling.hpp"
// Wait at the cell centre for three distinct coherent, valid frames.
class CellObservationGate {
public:
    static constexpr uint32_t TIMEOUT_MS=1500;
    void begin(uint32_t now){started_=now;lastSample_=0;count_=0;hasSample_=false;}
    void update(uint32_t sample,bool valid,uint32_t now){
        if(static_cast<int32_t>(sample-started_)<0)return;
        if(now-sample>SonarSampling::MAX_SAMPLE_AGE_MS){count_=0;return;}
        if(hasSample_&&sample==lastSample_){if(!valid)count_=0;return;}
        hasSample_=true;lastSample_=sample;
        count_=valid?(count_<3?count_+1:count_):0;
    }
    bool expired(uint32_t now)const{return static_cast<uint32_t>(now-started_)>=TIMEOUT_MS;}
    bool ready(uint32_t now)const{return !expired(now)&&hasSample_&&count_>=3&&
        now-lastSample_<=SonarSampling::MAX_SAMPLE_AGE_MS;}
private:
    uint32_t started_=0,lastSample_=0;
    uint8_t count_=0;
    bool hasSample_=false;
};
