#pragma once

#include <cstdint>

#include "config/maze_config.hpp"
#include "navigation/AutonomousNavigator.h"

namespace nav {

enum class MissionState : uint8_t {
    NotStarted,
    AwaitingObservation,
    Turning,
    Moving,
    Completed,
    Failed
};

enum class MissionAction : uint8_t {
    None,
    TurnLeft90,
    TurnRight90,
    TurnAround180,
    MoveForwardOneCell,
    Brake
};

struct MissionUpdate {
    MissionState state;
    MissionAction action;
    NavigationStatus navigationStatus;
};

class MazeMissionController {
public:
    MazeMissionController(mm::MazeMap &map, mm::IMapStorage &storage)
        : map_(map), navigator_(map, storage), state_(MissionState::NotStarted),
          x_(0), y_(0), heading_(mm::Dir::North), pendingDirection_(mm::Dir::North) {}

    /** Valida a configuração, inicializa o mapa e registra a origem e o objetivo. */
    bool begin(const MazeConfig &config);

    /** Fornece as três leituras da célula atual; só válido em AwaitingObservation. */
    MissionUpdate observeCell(const WallReadings &walls);

    /** Notifica conclusão/falha da ação de movimento atualmente solicitada. */
    MissionUpdate onMotionFinished(bool succeeded);

    /** Falha explícita da camada de hardware/sensores. */
    MissionUpdate fail();

    MissionState state() const { return state_; }
    uint8_t x() const { return x_; }
    uint8_t y() const { return y_; }
    mm::Dir heading() const { return heading_; }
    uint8_t goalX() const { return navigator_.goalX(); }
    uint8_t goalY() const { return navigator_.goalY(); }

private:
    MissionUpdate update(MissionAction action,
                         NavigationStatus status = NavigationStatus::NotInitialized) const;
    MissionUpdate failWith(NavigationStatus status);

    mm::MazeMap &map_;
    AutonomousNavigator navigator_;
    MazeConfig config_{};
    MissionState state_;
    uint8_t x_;
    uint8_t y_;
    mm::Dir heading_;
    mm::Dir pendingDirection_;
};

}  // namespace nav
