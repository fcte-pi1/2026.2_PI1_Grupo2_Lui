#pragma once
#include <cstdint>
#include <cmath>
#include "config/maze_geometry.hpp"
namespace nav {
// Keep 8cm of measured frontal space before changing to the 2cm/s approach.
// A cell-length step is a maximum, never permission to drive through a wall.
inline float guidedPreCornerMoveMm(float frontCm) {
    if (!std::isfinite(frontCm)) return 0;
    return std::fmin(static_cast<float>(MazeSettings::CELL_SIZE_MM), std::fmax(0.0f, (frontCm - 8.0f) * 10.0f));
}
enum class GuidedAction : uint8_t { ForwardCell, ApproachFrontWall, TurnLeft90, Complete, Failed };
// Relative movement test; deliberately independent of the maze solver/map.
class GuidedRoute {
public:
    void reset() { step_ = 0; failed_ = false; }
    void resetAtCorner() { reset(); step_ = 2; }
    GuidedAction action() const {
        if (failed_) return GuidedAction::Failed;
        if (step_ >= 5) return GuidedAction::Complete;
        if (step_ == 2) return GuidedAction::ApproachFrontWall;
        return step_ == 3 ? GuidedAction::TurnLeft90 : GuidedAction::ForwardCell;
    }
    void finishMovement(bool success) {
        if (action() == GuidedAction::Complete || failed_) return;
        if (!success) failed_ = true; else ++step_;
    }
    uint8_t step() const { return step_; }
private:
    uint8_t step_ = 0;
    bool failed_ = false;
};
} // namespace nav
