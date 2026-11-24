#pragma once
#include "maze_config.hpp"
#include "network_config.hpp"
namespace RunConfig {
// Shared with network_config.local.hpp; compile-time errors prevent silently
// running a different maze/heading than the operator requested.
static_assert(MM_MAZE_WIDTH == 4 || MM_MAZE_WIDTH == 8 || MM_MAZE_WIDTH == 12,
              "MM_MAZE_WIDTH must be 4, 8 or 12 (height is 4)");
static_assert(MM_START_CORNER >= 0 && MM_START_CORNER <= 3, "Invalid start corner");
static_assert(MM_INITIAL_HEADING >= 0 && MM_INITIAL_HEADING <= 3, "Invalid heading");
constexpr MazeFormat FORMAT = MM_MAZE_WIDTH == 4 ? MazeFormat::Maze4x4 :
                              MM_MAZE_WIDTH == 8 ? MazeFormat::Maze8x4 : MazeFormat::Maze12x4;
constexpr MazeCorner START = static_cast<MazeCorner>(MM_START_CORNER);
constexpr Direction HEADING = static_cast<Direction>(MM_INITIAL_HEADING);
constexpr uint32_t START_DELAY_MS = 5000;
// O robô é posicionado com a traseira encostada na parede do canto: o eixo fica
// ~44 mm atrás do centro da célula de largada (medido pelo sonar frontal no labirinto,
// 23/11). O primeiro avanço soma essa distância para as observações saírem do centro.
constexpr float START_BEHIND_CENTER_MM = 44.0f;
} // namespace RunConfig
