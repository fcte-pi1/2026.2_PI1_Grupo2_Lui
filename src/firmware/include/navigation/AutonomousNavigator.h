#pragma once
#include <cstdint>

#include "hal/IMapStorage.h"
#include "navigation/MazeMap.h"
#include "navigation/MazeSolver.h"

namespace nav {

enum class NavigationStatus : uint8_t {
    Move,               // nextDirection indica a célula adjacente a seguir
    AtGoal,              // posição observada é a célula objetivo
    NoPath,              // nenhuma rota possível com as paredes conhecidas
    NotInitialized,      // begin() ainda não foi concluído
    InvalidObservation,  // posição fora do mapa ou atualização inválida
    PersistenceError     // não foi possível salvar o mapa atualizado
};

struct WallReadings {
    bool front = false;
    bool left = false;
    bool right = false;
    // A valid false value means a measured open passage; invalid means unknown.
    bool frontValid = true;
    bool leftValid = true;
    bool rightValid = true;
};

struct NavigationResult {
    NavigationStatus status;
    mm::Dir nextDirection;  // direção absoluta; válida somente para Move
    uint16_t distance;      // passos estimados; Unreachable se não houver rota
};

class AutonomousNavigator {
public:
    AutonomousNavigator(mm::MazeMap &map, mm::IMapStorage &storage)
        : map_(map), solver_(map), storage_(storage), initialized_(false),
          hasLastPosition_(false), lastX_(0), lastY_(0), startX_(0), startY_(0),
          goalX_(0), goalY_(0) {}

    bool begin();

    bool begin(uint8_t startX, uint8_t startY, uint8_t goalX, uint8_t goalY);

    /**
     * Registra as paredes da célula atual, persiste o mapa e calcula a próxima
     * direção. Deve ser chamado uma vez por centro de célula, antes de mover.
     */
    NavigationResult observeCell(int x, int y, mm::Dir heading,
                                 const WallReadings &walls);

    bool initialized() const { return initialized_; }
    uint8_t startX() const { return startX_; }
    uint8_t startY() const { return startY_; }
    uint8_t goalX() const { return goalX_; }
    uint8_t goalY() const { return goalY_; }
    const mm::MazeMap &map() const { return map_; }

private:
    NavigationResult result(NavigationStatus status, mm::Dir next,
                            uint16_t distance) const {
        NavigationResult r = { status, next, distance };
        return r;
    }

    mm::MazeMap &map_;
    MazeSolver solver_;
    mm::IMapStorage &storage_;
    bool initialized_;
    bool hasLastPosition_;
    uint8_t lastX_;
    uint8_t lastY_;
    uint8_t startX_;
    uint8_t startY_;
    uint8_t goalX_;
    uint8_t goalY_;
};

}  // namespace nav
