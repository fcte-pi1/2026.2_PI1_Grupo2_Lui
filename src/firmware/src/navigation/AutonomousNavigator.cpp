#include "navigation/AutonomousNavigator.h"

namespace nav {

bool AutonomousNavigator::begin()
{
    if (map_.width() == 0 || map_.height() == 0) {
        initialized_ = false;
        return false;
    }
    return begin(0, 0,
                 static_cast<uint8_t>(map_.width() - 1),
                 static_cast<uint8_t>(map_.height() - 1));
}

bool AutonomousNavigator::begin(uint8_t startX, uint8_t startY,
                                uint8_t goalX, uint8_t goalY)
{
    initialized_ = false;
    hasLastPosition_ = false;
    if (map_.get(startX, startY) == nullptr ||
        map_.get(goalX, goalY) == nullptr) {
        return false;
    }
    if (!solver_.setGoal(goalX, goalY)) {
        return false;
    }
    startX_ = startX;
    startY_ = startY;
    goalX_ = goalX;
    goalY_ = goalY;
    initialized_ = true;
    return true;
}

NavigationResult AutonomousNavigator::observeCell(
    int x, int y, mm::Dir heading, const WallReadings &walls)
{
    if (!initialized_) {
        return result(NavigationStatus::NotInitialized, heading,
                      MazeSolver::kUnreachable);
    }

    if (map_.get(x, y) == nullptr) {
        return result(NavigationStatus::InvalidObservation, heading,
                      MazeSolver::kUnreachable);
    }

    // A new observation must be in the same cell (retry) or an adjacent cell
    // (a completed one-cell motion). Reject jumps so a localization fault does
    // not create false-open corridors in the map.
    if (hasLastPosition_ && (x != lastX_ || y != lastY_)) {
        const int dx = x - static_cast<int>(lastX_);
        const int dy = y - static_cast<int>(lastY_);
        mm::Dir traversed;
        if (dx == 1 && dy == 0) traversed = mm::Dir::East;
        else if (dx == -1 && dy == 0) traversed = mm::Dir::West;
        else if (dx == 0 && dy == 1) traversed = mm::Dir::South;
        else if (dx == 0 && dy == -1) traversed = mm::Dir::North;
        else {
            return result(NavigationStatus::InvalidObservation, heading,
                          MazeSolver::kUnreachable);
        }
        // The robot physically reached the neighbouring cell, so this edge is
        // known to be open even when both cells' sensors cannot see it (rear).
        map_.setWall(lastX_, lastY_, traversed, false);
    }

    if (!map_.updateCell(x, y, heading, walls.front, walls.left, walls.right)) {
        return result(NavigationStatus::InvalidObservation, heading,
                      MazeSolver::kUnreachable);
    }
    hasLastPosition_ = true;
    lastX_ = static_cast<uint8_t>(x);
    lastY_ = static_cast<uint8_t>(y);

    // Persist before returning a movement command. If storage fails, do not
    // authorize another movement with a map state that was not saved.
    if (!map_.save(storage_)) {
        return result(NavigationStatus::PersistenceError, heading,
                      MazeSolver::kUnreachable);
    }

    mm::Dir next = heading;
    const Decision decision = solver_.decide(x, y, heading, next);
    const uint16_t distance = solver_.distance(x, y);

    switch (decision) {
    case Decision::Move:
        return result(NavigationStatus::Move, next, distance);
    case Decision::AtGoal:
        return result(NavigationStatus::AtGoal, heading, 0);
    case Decision::NoPath:
    default:
        return result(NavigationStatus::NoPath, heading,
                      MazeSolver::kUnreachable);
    }
}

}  // namespace nav
