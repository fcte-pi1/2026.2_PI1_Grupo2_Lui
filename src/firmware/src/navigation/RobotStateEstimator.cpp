#include "navigation/RobotStateEstimator.h"

#include <cstddef>

namespace state_estimation {
namespace {

constexpr std::size_t indexOf(tracking::Orientation direction) {
    return static_cast<std::size_t>(direction);
}

void setObservation(RobotStateSnapshot& snapshot,
                    tracking::Orientation absoluteDirection,
                    bool valid,
                    bool wallPresent,
                    uint16_t distanceMm) {
    AbsoluteWallObservation& observation = snapshot.walls[indexOf(absoluteDirection)];
    if (!valid) {
        observation = AbsoluteWallObservation{};
        return;
    }

    observation.knowledge = wallPresent ? WallKnowledge::Present : WallKnowledge::Open;
    observation.distanceMm = distanceMm;
    observation.distanceValid = true;
}

}  // namespace

const AbsoluteWallObservation& RobotStateSnapshot::wallAt(tracking::Orientation direction) const {
    return walls[indexOf(direction)];
}

RobotStateEstimator::RobotStateEstimator(tracking::PositionTracker& positionTracker,
                                         perception::WallPerception& wallPerception)
    : positionTracker_(positionTracker), wallPerception_(wallPerception) {}

void RobotStateEstimator::initialize(const tracking::TrackingState& initialPose,
                                     int16_t initialYawReferenceDegrees) {
    positionTracker_.reset(initialPose, initialYawReferenceDegrees);
    state_ = RobotStateSnapshot{};
    state_.pose = initialPose;
}

void RobotStateEstimator::updatePose() {
    positionTracker_.update();
    state_.pose = positionTracker_.currentState();
}

void RobotStateEstimator::updateWalls() {
    wallPerception_.update();
    state_.pose = positionTracker_.currentState();
    const perception::WallCellState& walls = wallPerception_.currentState();

    // Não há sensor traseiro no contrato da HU-01. Resetar os quatro slots a
    // cada ciclo mantém essa direção como Unknown, em vez de inferir caminho livre.
    state_.walls.fill(AbsoluteWallObservation{});

    const tracking::Orientation front = state_.pose.orientation;
    const tracking::Orientation left = tracking::rotateOrientation(front, 1, false);
    const tracking::Orientation right = tracking::rotateOrientation(front, 1, true);

    setObservation(state_, front, walls.frontValid, walls.wallFront, walls.distFrontMm);
    setObservation(state_, left, walls.leftValid, walls.wallLeft, walls.distLeftMm);
    setObservation(state_, right, walls.rightValid, walls.wallRight, walls.distRightMm);

    state_.sensorTimestampMs = walls.timestampMs;
    state_.initialized = true;
}

void RobotStateEstimator::update() {
    updatePose();
    updateWalls();
}

}  // namespace state_estimation
