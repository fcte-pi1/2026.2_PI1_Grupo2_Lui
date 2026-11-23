#pragma once
#include <cstdint>

// Só a manobra proprietária da suspensão pode retomar a amostragem.
class CornerSonarGate {
public:
    void suspend(uint32_t now){suspended_=true;requestedAt_=now;}
    void resume(uint32_t now){suspended_=false;hasResume_=true;resumedAt_=now;}
    bool suspended() const {return suspended_;}
    bool pauseExpired(uint32_t now) const {return suspended_ && uint32_t(now-requestedAt_)>=300;}
    bool accepts(uint32_t sampledAt) const {
        return !suspended_ && (!hasResume_ || int32_t(sampledAt-resumedAt_)>=0);
    }
private:
    bool suspended_=false,hasResume_=false;
    uint32_t requestedAt_=0,resumedAt_=0;
};
