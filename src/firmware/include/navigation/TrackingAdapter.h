#pragma once

/**
 * Adaptador PositionTracker (HU-02) -> MazeMap (HU-00).
 * x = col, y = row; tracking::Orientation e mm::Dir têm a mesma ordem (N,E,S,W).
 */

#include <cstdint>

#include "navigation/MazeMap.h"
#include "navigation/PositionTracker.h"

namespace mm {

static_assert(static_cast<uint8_t>(tracking::Orientation::North) == static_cast<uint8_t>(Dir::North) &&
              static_cast<uint8_t>(tracking::Orientation::East)  == static_cast<uint8_t>(Dir::East)  &&
              static_cast<uint8_t>(tracking::Orientation::South) == static_cast<uint8_t>(Dir::South) &&
              static_cast<uint8_t>(tracking::Orientation::West)  == static_cast<uint8_t>(Dir::West),
              "tracking::Orientation e mm::Dir devem ter a mesma ordem");

inline Dir toDir(tracking::Orientation o)
{
    return static_cast<Dir>(static_cast<uint8_t>(o));
}

/**
 * Atualiza o mapa na célula atual do tracker com as leituras binárias de
 * parede (vindas da HU-01) relativas ao robô.
 */
inline bool updateMapFromTracking(MazeMap &map, const tracking::TrackingState &state,
                                  bool wall_front, bool wall_left, bool wall_right)
{
    return map.updateCell(state.position.col, state.position.row,
                          toDir(state.orientation), wall_front, wall_left, wall_right);
}

}  // namespace mm
