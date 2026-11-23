#pragma once
#include "front_proximity_guard.hpp"
#include "../hal/sonar_snapshot.hpp"

// Uma leitura lateral curta freia para conferir; duas em três confirmam bloqueio.
// Esta política não comanda ré nem altera o alvo do avanço.
class StraightProximityGuard {
public:
    struct Feedback {bool pause=false,leftConfirmed=false,rightConfirmed=false;};
    Feedback update(const SonarSnapshot& frame) {
        const auto near=[](const UltrasonicReading& value){
            return value.tooClose || (value.valid && value.distanceCm<=1.9f);
        };
        const auto l=left_.update(near(frame.left),frame.sampledAtMs);
        const auto r=right_.update(near(frame.right),frame.sampledAtMs);
        return {l==FrontProximityGuard::State::Suspect || r==FrontProximityGuard::State::Suspect,
                l==FrontProximityGuard::State::Confirmed,r==FrontProximityGuard::State::Confirmed};
    }
    void reset(){left_.reset();right_.reset();}
private:
    FrontProximityGuard left_,right_;
};
