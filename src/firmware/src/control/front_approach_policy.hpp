#pragma once
#include <cmath>
#include <cstdint>
#include "../config/maze_geometry.hpp"

namespace FrontApproach {
constexpr float START_CM=12.0f;
constexpr float TARGET_CM=MazeSettings::GUIDED_FRONT_TARGET_CM;
constexpr float TOLERANCE_CM=MazeSettings::GUIDED_FRONT_TOLERANCE_CM;
constexpr float EMERGENCY_CM=MazeSettings::GUIDED_FRONT_EMERGENCY_CM;
constexpr float MAX_SPEED_MM_S=20.0f;
constexpr float SLOW_MEASURED_MM_S=30.0f;
constexpr float POSITION_TOLERANCE_MM=20.0f;
constexpr uint32_t TIMEOUT_MS=12000;
inline bool shouldSlow(bool valid,float cm){return valid&&std::isfinite(cm)&&cm<=START_CM;}
inline bool atTarget(bool valid,float cm){return valid&&std::isfinite(cm)&&cm<=TARGET_CM+TOLERANCE_CM;}
inline bool slowEnough(float left,float right){return std::isfinite(left)&&std::isfinite(right)&&
    std::fabs(left)<=SLOW_MEASURED_MM_S&&std::fabs(right)<=SLOW_MEASURED_MM_S;}
inline bool positionMatchesCell(float travelled,float cell){return std::isfinite(travelled)&&std::isfinite(cell)&&
    std::fabs(travelled-cell)<=POSITION_TOLERANCE_MM;}
inline bool confirmed(bool valid,float cm,float travelled,float cell){return valid&&std::isfinite(cm)&&
    std::fabs(cm-TARGET_CM)<=TOLERANCE_CM&&positionMatchesCell(travelled,cell);}
inline bool arrivalConfirmed(bool usedFrontReference,bool valid,float cm,float travelled,float cell){
    return positionMatchesCell(travelled,cell)&&(!usedFrontReference||confirmed(valid,cm,travelled,cell));
}
}
