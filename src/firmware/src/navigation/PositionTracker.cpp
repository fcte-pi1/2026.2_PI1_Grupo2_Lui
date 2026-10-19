#include "navigation/PositionTracker.h"

#include <cstdlib>

namespace tracking {

namespace {

int16_t absoluteValue(int16_t value) {
    return static_cast<int16_t>(value < 0 ? -value : value);
}

// O sensor reporta yaw circular em 0..359 graus. Normalizar a diferença
// evita interpretar, por exemplo, 350 -> 80 graus como um giro de -270 graus.
int16_t shortestYawDelta(int16_t currentDegrees, int16_t referenceDegrees) {
    int16_t delta = static_cast<int16_t>(currentDegrees - referenceDegrees);
    while (delta > 180) {
        delta = static_cast<int16_t>(delta - 360);
    }
    while (delta < -180) {
        delta = static_cast<int16_t>(delta + 360);
    }
    return delta;
}

}  // namespace

Orientation rotateOrientation(Orientation current, uint8_t steps, bool clockwise) {
    const int8_t signedSteps = clockwise ? static_cast<int8_t>(steps) : -static_cast<int8_t>(steps);
    const int8_t currentIndex = static_cast<int8_t>(current);
    int8_t nextIndex = static_cast<int8_t>((currentIndex + signedSteps) % 4);
    if (nextIndex < 0) {
        nextIndex += 4;
    }
    return static_cast<Orientation>(nextIndex);
}

GridPosition applyCellAdvance(GridPosition current, Orientation orientation) {
    switch (orientation) {
        case Orientation::North:
            current.row -= 1;
            break;
        case Orientation::South:
            current.row += 1;
            break;
        case Orientation::East:
            current.col += 1;
            break;
        case Orientation::West:
            current.col -= 1;
            break;
    }
    return current;
}

bool tryConfirmRotation(int16_t yawDeltaDegrees, uint8_t& outSteps, bool& outClockwise) {
    const int16_t magnitude = absoluteValue(yawDeltaDegrees);
    const int16_t distanceTo90 = absoluteValue(static_cast<int16_t>(magnitude - 90));
    const int16_t distanceTo180 = absoluteValue(static_cast<int16_t>(magnitude - 180));

    if (distanceTo90 <= kRotationToleranceDegrees) {
        outSteps = 1;
        outClockwise = yawDeltaDegrees > 0;
        return true;
    }
    if (distanceTo180 <= kRotationToleranceDegrees) {
        outSteps = 2;
        outClockwise = yawDeltaDegrees > 0;
        return true;
    }
    return false;
}

PositionTracker::PositionTracker(IOdometryReader& odometry, IOrientationSensor& orientationSensor)
    : PositionTracker(odometry, orientationSensor, TrackingState{}, 0) {}

PositionTracker::PositionTracker(IOdometryReader& odometry,
                                 IOrientationSensor& orientationSensor,
                                 const TrackingState& initialState,
                                 int16_t initialYawReferenceDegrees)
    : odometry_(odometry),
      orientationSensor_(orientationSensor),
      state_(initialState),
      movementYawReferenceDegrees_(initialYawReferenceDegrees) {}

void PositionTracker::update() {
    const OrientationReading yawReading = orientationSensor_.read();
    const int16_t yawDelta = shortestYawDelta(yawReading.yawDegrees, movementYawReferenceDegrees_);

    uint8_t rotationSteps = 0;
    bool rotationClockwise = false;
    if (tryConfirmRotation(yawDelta, rotationSteps, rotationClockwise)) {
        state_.orientation = rotateOrientation(state_.orientation, rotationSteps, rotationClockwise);
        movementYawReferenceDegrees_ = yawReading.yawDegrees;
    }

    const OdometryReading odoReading = odometry_.read();

    if (odoReading.accumulatedDistanceMm >= kCellDistanceThresholdMm) {
        const uint32_t elapsedSinceLastAdvance = odoReading.timestampMs - lastAdvanceAcceptedMs_;
        const bool withinDedupWindow =
            hasAcceptedAdvance_ && elapsedSinceLastAdvance < kMinCellAdvanceIntervalMs;

        if (!withinDedupWindow) {
            state_.position = applyCellAdvance(state_.position, state_.orientation);
            lastAdvanceAcceptedMs_ = odoReading.timestampMs;
            hasAcceptedAdvance_ = true;
        }
        odometry_.resetAccumulator();
    }

    state_.timestampMs = odoReading.timestampMs;
}

}  // namespace tracking
