#pragma once

#include <cstdint>

// Geometria física compartilhada pelos módulos de configuração e percepção.
// Deve ser calibrada conforme o padrão real do labirinto e o chassi.
namespace MazeSettings {
constexpr uint16_t CLEAR_CELL_MM = 170; // vao livre confirmado em 16/11
constexpr uint16_t WALL_THICKNESS_MM = 10;
constexpr uint16_t CELL_SIZE_MM = CLEAR_CELL_MM + WALL_THICKNESS_MM;
constexpr float GUIDED_FRONT_TARGET_CM = 3.0f;
constexpr float GUIDED_FRONT_TOLERANCE_CM = 0.5f;
constexpr float GUIDED_FRONT_EMERGENCY_CM = 2.0f;
constexpr float WALL_THRESHOLD_CELL_RATIO = 0.55f;
}  // namespace MazeSettings
