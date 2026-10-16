#pragma once

/**
 * Integração HU-00 (mapa interno) + HU-03 (decisão Flood Fill).
 *
 * O controlador deve ser chamado quando o robô estiver parado no centro de
 * uma célula, com a posição/orientação atualizadas pelo subsistema de
 * localização e com as leituras binárias da HU-01 disponíveis.
 *
 * O controlador atualiza o mapa, persiste a versão mais recente e só então
 * pede ao solver a próxima direção absoluta. O código de movimento (HU-04 /
 * HU-05) executa essa direção e chama observeCell novamente ao chegar à
 * próxima célula. Isso mantém os drivers de hardware fora da lógica pura.
 */

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
    bool front;  // true se existe parede à frente
    bool left;   // true se existe parede à esquerda
    bool right;  // true se existe parede à direita
};

struct NavigationResult {
    NavigationStatus status;
    mm::Dir nextDirection;  // direção absoluta; válida somente para Move
    uint16_t distance;      // passos estimados; kUnreachable se não houver rota
};

class AutonomousNavigator {
public:
    AutonomousNavigator(mm::MazeMap &map, mm::IMapStorage &storage)
        : map_(map), solver_(map), storage_(storage), initialized_(false),
          hasLastPosition_(false), lastX_(0), lastY_(0) {}

    /** Configura como objetivo o canto oposto: (width-1, height-1). */
    bool begin();

    /**
     * Registra as paredes da célula atual, persiste o mapa e calcula a próxima
     * direção. Deve ser chamado uma vez por centro de célula, antes de mover.
     */
    NavigationResult observeCell(int x, int y, mm::Dir heading,
                                 const WallReadings &walls);

    bool initialized() const { return initialized_; }
    uint8_t goalX() const { return map_.width() ? map_.width() - 1 : 0; }
    uint8_t goalY() const { return map_.height() ? map_.height() - 1 : 0; }
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
};

}  // namespace nav
