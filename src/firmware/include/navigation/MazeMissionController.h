#pragma once

#include <cstdint>

#include "config/maze_config.hpp"
#include "navigation/AutonomousNavigator.h"
#include "navigation/RobotStateEstimator.h"

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
    MoveBackwardOneCell,
    Brake
};

struct MissionUpdate {
    MissionState state;
    MissionAction action;
    NavigationStatus navigationStatus;
};

// Orquestra HU-00/03/05/06 usando o snapshot HU-01/02 como fonte única de
// pose e observações. O controlador não mantém uma segunda cópia de x/y/heading.
class MazeMissionController {
public:
    MazeMissionController(mm::MazeMap& map, mm::IMapStorage& storage,
                          state_estimation::RobotStateEstimator& stateEstimator)
        : map_(map), navigator_(map, storage), stateEstimator_(stateEstimator),
          state_(MissionState::NotStarted), pendingDirection_(mm::Dir::North),
          expectedX_(0), expectedY_(0) {}

    bool begin(const MazeConfig& config, int16_t initialYawReferenceDegrees = 0);

    // Usa pose e paredes absolutas mais recentes produzidas por RobotStateEstimator.
    MissionUpdate observeCurrentCell();
    MissionUpdate onMotionFinished(bool succeeded);
    MissionUpdate fail();

    MissionState state() const { return state_; }
    uint8_t x() const;
    uint8_t y() const;
    mm::Dir heading() const;
    uint8_t goalX() const { return navigator_.goalX(); }
    uint8_t goalY() const { return navigator_.goalY(); }

private:
    MissionUpdate update(MissionAction action,
                         NavigationStatus status = NavigationStatus::NotInitialized) const;
    MissionUpdate failWith(NavigationStatus status);
    bool currentPoseInBounds() const;

    mm::MazeMap& map_;
    AutonomousNavigator navigator_;
    state_estimation::RobotStateEstimator& stateEstimator_;
    MazeConfig config_{};
    MissionState state_;
    mm::Dir pendingDirection_;
    uint8_t expectedX_;
    uint8_t expectedY_;
};

}  // namespace nav
