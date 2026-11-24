#pragma once
#include <cstdint>
#include "../config/sonar_sampling.hpp"
// Brake on the first suspect short echo. End the mission only after two
// short echoes in three distinct frames; repeated control ticks don't vote.
class FrontProximityGuard {
public:
    enum class State{Clear,Suspect,Confirmed};
    State update(bool near,uint32_t sampledAt){
        if(hasSample_&&sampledAt==lastSample_)return state_;
        if(hasSample_&&sampledAt-lastSample_>SonarSampling::MAX_SAMPLE_AGE_MS)reset();
        lastSample_=sampledAt;hasSample_=true;
        votes_[cursor_]=near;cursor_=(cursor_+1)%3;
        const unsigned count=unsigned(votes_[0])+unsigned(votes_[1])+unsigned(votes_[2]);
        state_=count>=2?State::Confirmed:near?State::Suspect:State::Clear;
        return state_;
    }
    void reset(){*this=FrontProximityGuard{};}
private:
    bool votes_[3]{};
    uint8_t cursor_=0;
    bool hasSample_=false;
    uint32_t lastSample_=0;
    State state_=State::Clear;
};
