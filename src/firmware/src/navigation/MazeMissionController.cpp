#include "navigation/MazeMissionController.h"

namespace nav {

MissionUpdate MazeMissionController::update(MissionAction action,
                                             NavigationStatus status) const
{
    return {state_, action, status};
}

MissionUpdate MazeMissionController::failWith(NavigationStatus status)
{
    state_ = MissionState::Failed;
    return update(MissionAction::Brake, status);
}

bool MazeMissionController::begin(const MazeConfig &config)
{
    state_ = MissionState::NotStarted;
    config_ = config;
    if (!isValidMazeConfig(config_)) {
        return false;
    }
    if (!map_.init(config_.cols, config_.rows)) {
        return false;
    }

    x_ = config_.start.col;
    y_ = config_.start.row;
    heading_ = static_cast<mm::Dir>(config_.initialHeading);
    pendingDirection_ = heading_;

    if (!navigator_.begin(config_.start.col, config_.start.row,
                          config_.goal.col, config_.goal.row)) {
        state_ = MissionState::Failed;
        return false;
    }
    state_ = MissionState::AwaitingObservation;
    return true;
}

MissionUpdate MazeMissionController::observeCell(const WallReadings &walls)
{
    if (state_ != MissionState::AwaitingObservation) {
        return failWith(NavigationStatus::InvalidObservation);
    }

    const NavigationResult navigation = navigator_.observeCell(x_, y_, heading_, walls);
    switch (navigation.status) {
    case NavigationStatus::AtGoal:
        state_ = MissionState::Completed;
        return update(MissionAction::Brake, navigation.status);

    case NavigationStatus::NoPath:
    case NavigationStatus::NotInitialized:
    case NavigationStatus::InvalidObservation:
    case NavigationStatus::PersistenceError:
        return failWith(navigation.status);

    case NavigationStatus::Move:
        break;
    }

    pendingDirection_ = navigation.nextDirection;
    const uint8_t current = static_cast<uint8_t>(heading_);
    const uint8_t next = static_cast<uint8_t>(pendingDirection_);
    const uint8_t turn = static_cast<uint8_t>((next + 4u - current) & 0x3u);

    if (turn == 0) {
        state_ = MissionState::Moving;
        return update(MissionAction::MoveForwardOneCell, navigation.status);
    }
    state_ = MissionState::Turning;
    if (turn == 1) {
        return update(MissionAction::TurnRight90, navigation.status);
    }
    if (turn == 3) {
        return update(MissionAction::TurnLeft90, navigation.status);
    }
    return update(MissionAction::TurnAround180, navigation.status);
}

MissionUpdate MazeMissionController::onMotionFinished(bool succeeded)
{
    if (!succeeded) {
        return failWith(NavigationStatus::InvalidObservation);
    }

    if (state_ == MissionState::Turning) {
        heading_ = pendingDirection_;
        state_ = MissionState::Moving;
        return update(MissionAction::MoveForwardOneCell);
    }

    if (state_ != MissionState::Moving) {
        return failWith(NavigationStatus::InvalidObservation);
    }

    switch (pendingDirection_) {
    case mm::Dir::North:
        if (y_ == 0) return failWith(NavigationStatus::InvalidObservation);
        --y_;
        break;
    case mm::Dir::East:
        if (x_ + 1 >= config_.cols) return failWith(NavigationStatus::InvalidObservation);
        ++x_;
        break;
    case mm::Dir::South:
        if (y_ + 1 >= config_.rows) return failWith(NavigationStatus::InvalidObservation);
        ++y_;
        break;
    case mm::Dir::West:
        if (x_ == 0) return failWith(NavigationStatus::InvalidObservation);
        --x_;
        break;
    }
    state_ = MissionState::AwaitingObservation;
    return update(MissionAction::None);
}

MissionUpdate MazeMissionController::fail()
{
    return failWith(NavigationStatus::InvalidObservation);
}

}  // namespace nav
