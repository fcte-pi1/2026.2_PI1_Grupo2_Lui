#pragma once
#include <cstdint>

// Called only after stale/near-obstacle checks. Missing echo never means open.
class FrontEchoRecovery {
public:
    enum class Decision { Proceed, Hold, Failed };
    static constexpr uint32_t TIMEOUT_MS=1200;
    static constexpr uint32_t MIN_HOLD_MS=200;
    static constexpr uint8_t REQUIRED_GOOD_FRAMES=2;
    static constexpr uint8_t MAX_EPISODES=3;
    void reset(){*this=FrontEchoRecovery{};}
    Decision update(bool valid, uint32_t sampledAt, uint32_t now, bool settled) {
        if(failed_)return Decision::Failed;
        if(!holding_) {
            if(valid)return Decision::Proceed;
            if(episodes_>=MAX_EPISODES){failed_=true;return Decision::Failed;}
            ++episodes_;holding_=true;startedAt_=now;goodFrames_=0;
            lastSample_=sampledAt;
        }
        if(static_cast<uint32_t>(now-startedAt_)>=TIMEOUT_MS){failed_=true;return Decision::Failed;}
        if(sampledAt!=lastSample_) {
            lastSample_=sampledAt;
            goodFrames_=valid ? (goodFrames_<REQUIRED_GOOD_FRAMES?goodFrames_+1:goodFrames_) : 0;
        } else if(!valid)goodFrames_=0;
        if(valid && goodFrames_>=REQUIRED_GOOD_FRAMES && settled &&
           static_cast<uint32_t>(now-startedAt_)>=MIN_HOLD_MS) {
            holding_=false;return Decision::Proceed;
        }
        return Decision::Hold;
    }
    bool holding()const{return holding_;}
    uint8_t episodes()const{return episodes_;}
private:
    bool holding_=false,failed_=false;
    uint32_t startedAt_=0,lastSample_=0;
    uint8_t goodFrames_=0,episodes_=0;
};
