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

void RobotStateEstimator::update() {
    // O rastreador é atualizado primeiro para que as leituras relativas de
    // frente/esquerda/direita sejam projetadas na orientação mais recente.
    positionTracker_.update();
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

}  // namespace state_estimation
