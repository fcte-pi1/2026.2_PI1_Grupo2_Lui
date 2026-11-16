#pragma once
#include <cstdint>
namespace nav {
enum class GuidedAction : uint8_t { ForwardCell, TurnLeft90, Complete, Failed };
// Relative movement test; deliberately independent of the maze solver/map.
class GuidedRoute {
public:
    void reset() { step_ = 0; failed_ = false; }
    void resetAtCorner() { reset(); step_ = 3; }
    GuidedAction action() const {
        if (failed_) return GuidedAction::Failed;
        if (step_ >= 5) return GuidedAction::Complete;
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
