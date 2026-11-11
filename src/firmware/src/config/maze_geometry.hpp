#pragma once

#include <cstdint>

// Geometria física compartilhada pelos módulos de configuração e percepção.
// Deve ser calibrada conforme o padrão real do labirinto e o chassi.
namespace MazeSettings {
constexpr uint16_t CELL_SIZE_MM = 180;
constexpr float WALL_THRESHOLD_CELL_RATIO = 0.55f;
}  // namespace MazeSettings
