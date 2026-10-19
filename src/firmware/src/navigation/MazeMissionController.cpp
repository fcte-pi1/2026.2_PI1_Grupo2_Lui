#include "navigation/MazeMissionController.h"

namespace nav {

MissionUpdate MazeMissionController::update(MissionAction action,
                                             NavigationStatus status) const {
    return {state_, action, status};
}

MissionUpdate MazeMissionController::failWith(NavigationStatus status) {
    state_ = MissionState::Failed;
    return update(MissionAction::Brake, status);
}

bool MazeMissionController::begin(const MazeConfig& config,
                                  int16_t initialYawReferenceDegrees) {
    state_ = MissionState::NotStarted;
    config_ = config;
    if (!isValidMazeConfig(config_) || !map_.init(config_.cols, config_.rows)) {
        return false;
    }

    if (!navigator_.begin(config_.start.col, config_.start.row,
                          config_.goal.col, config_.goal.row)) {
        state_ = MissionState::Failed;
        return false;
    }

    const tracking::TrackingState initialPose{
        tracking::GridPosition{config_.start.row, config_.start.col},
        static_cast<tracking::Orientation>(config_.initialHeading),
        0
    };
    stateEstimator_.initialize(initialPose, initialYawReferenceDegrees);
    pendingDirection_ = static_cast<mm::Dir>(config_.initialHeading);
    expectedX_ = config_.start.col;
    expectedY_ = config_.start.row;
    state_ = MissionState::AwaitingObservation;
    return true;
}

bool MazeMissionController::currentPoseInBounds() const {
    const auto& pose = stateEstimator_.currentState().pose.position;
    return pose.row >= 0 && pose.col >= 0 &&
           pose.row < config_.rows && pose.col < config_.cols;
}

uint8_t MazeMissionController::x() const {
    const auto col = stateEstimator_.currentState().pose.position.col;
    return col < 0 ? 255 : static_cast<uint8_t>(col);
}

uint8_t MazeMissionController::y() const {
    const auto row = stateEstimator_.currentState().pose.position.row;
    return row < 0 ? 255 : static_cast<uint8_t>(row);
}

mm::Dir MazeMissionController::heading() const {
    return static_cast<mm::Dir>(stateEstimator_.currentState().pose.orientation);
}

MissionUpdate MazeMissionController::observeCurrentCell() {
    if (state_ != MissionState::AwaitingObservation) {
        return failWith(NavigationStatus::InvalidObservation);
    }
    const auto& snapshot = stateEstimator_.currentState();
    if (!snapshot.initialized || !currentPoseInBounds()) {
        return failWith(NavigationStatus::InvalidObservation);
    }

    const mm::Dir absoluteHeading = heading();
    const auto leftOrientation = tracking::rotateOrientation(
        snapshot.pose.orientation, 1, false);
    const auto rightOrientation = tracking::rotateOrientation(
        snapshot.pose.orientation, 1, true);
    const auto& front = snapshot.wallAt(snapshot.pose.orientation);
    const auto& left = snapshot.wallAt(leftOrientation);
    const auto& right = snapshot.wallAt(rightOrientation);

    WallReadings walls{};
    walls.front = front.knowledge == state_estimation::WallKnowledge::Present;
    walls.left = left.knowledge == state_estimation::WallKnowledge::Present;
    walls.right = right.knowledge == state_estimation::WallKnowledge::Present;
    walls.frontValid = front.knowledge != state_estimation::WallKnowledge::Unknown;
    walls.leftValid = left.knowledge != state_estimation::WallKnowledge::Unknown;
    walls.rightValid = right.knowledge != state_estimation::WallKnowledge::Unknown;

    const NavigationResult navigation = navigator_.observeCell(
        x(), y(), absoluteHeading, walls);
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

    // O Flood Fill pode tratar lados desconhecidos como candidatos otimistas.
    // Não se move para um lado sem observação válida/aresta já percorrida:
    // isso evita interpretar uma falha de sensor como corredor livre.
    const mm::Cell* currentCell = map_.get(x(), y());
    if (currentCell == nullptr || !currentCell->isKnown(navigation.nextDirection) ||
        currentCell->hasWall(navigation.nextDirection)) {
        return failWith(NavigationStatus::InvalidObservation);
    }

    pendingDirection_ = navigation.nextDirection;
    int expectedX = x();
    int expectedY = y();
    switch (pendingDirection_) {
    case mm::Dir::North: --expectedY; break;
    case mm::Dir::East:  ++expectedX; break;
    case mm::Dir::South: ++expectedY; break;
    case mm::Dir::West:  --expectedX; break;
    }
    if (expectedX < 0 || expectedY < 0 ||
        expectedX >= config_.cols || expectedY >= config_.rows) {
        return failWith(NavigationStatus::InvalidObservation);
    }
    expectedX_ = static_cast<uint8_t>(expectedX);
    expectedY_ = static_cast<uint8_t>(expectedY);

    const uint8_t currentDirection = static_cast<uint8_t>(absoluteHeading);
    const uint8_t nextDirection = static_cast<uint8_t>(pendingDirection_);
    const uint8_t turn = static_cast<uint8_t>((nextDirection + 4u - currentDirection) & 0x3u);
    if (turn == 0) {
        state_ = MissionState::Moving;
        return update(MissionAction::MoveForwardOneCell, navigation.status);
    }
    if (turn == 2) {
        // A HU-05 implementa movimento reverso assinado; usar a ré evita um
        // giro desnecessário de 180° e mantém a orientação medida pela IMU.
        state_ = MissionState::Moving;
        return update(MissionAction::MoveBackwardOneCell, navigation.status);
    }
    state_ = MissionState::Turning;
    if (turn == 1) return update(MissionAction::TurnRight90, navigation.status);
    if (turn == 3) return update(MissionAction::TurnLeft90, navigation.status);
    return update(MissionAction::TurnAround180, navigation.status);
}

MissionUpdate MazeMissionController::onMotionFinished(bool succeeded) {
    if (!succeeded) return failWith(NavigationStatus::InvalidObservation);

    if (state_ == MissionState::Turning) {
        if (heading() != pendingDirection_) {
            return failWith(NavigationStatus::InvalidObservation);
        }
        state_ = MissionState::Moving;
        return update(MissionAction::MoveForwardOneCell);
    }

    if (state_ != MissionState::Moving) {
        return failWith(NavigationStatus::InvalidObservation);
    }

    // PositionTracker (HU-02) deve confirmar a chegada; a missão nunca incrementa
    // x/y por conta própria, evitando drift entre duas fontes de pose.
    if (!currentPoseInBounds() || x() != expectedX_ || y() != expectedY_) {
        return failWith(NavigationStatus::InvalidObservation);
    }
    state_ = MissionState::AwaitingObservation;
    return update(MissionAction::None);
}

MissionUpdate MazeMissionController::fail() {
    return failWith(NavigationStatus::InvalidObservation);
}

}  // namespace nav
