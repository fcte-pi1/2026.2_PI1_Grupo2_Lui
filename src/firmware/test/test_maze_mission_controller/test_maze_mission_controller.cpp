// Integração nativa HU-00/01/02/03/05/06: pio test -e native -f test_maze_mission_controller
#include <unity.h>
#include <cstring>

#include "config/maze_config.hpp"
#include "navigation/MazeMissionController.h"
#include "navigation/PositionTracker.h"
#include "navigation/RobotStateEstimator.h"
#include "../test_position_tracker/mock_odometry_reader.h"
#include "../test_position_tracker/mock_orientation_sensor.h"
#include "../test_wall_perception/mock_wall_sensor.h"

namespace {

class FakeStorage final : public mm::IMapStorage {
public:
    uint8_t data[mm::MazeMap::kBlobMaxSize]{};
    size_t length = 0;
    bool failSave = false;

    bool save(const uint8_t* input, size_t size) override {
        if (failSave || size > sizeof(data)) return false;
        std::memcpy(data, input, size);
        length = size;
        return true;
    }
    size_t load(uint8_t* output, size_t capacity) override {
        if (length == 0 || length > capacity) return 0;
        std::memcpy(output, data, length);
        return length;
    }
};

class MissionFixture {
public:
    FakeStorage storage;
    tracking::test::MockOdometryReader odometry;
    tracking::test::MockOrientationSensor orientation;
    perception::test::MockWallSensor wallSensor;
    tracking::PositionTracker tracker;
    perception::WallPerception wallPerception;
    state_estimation::RobotStateEstimator estimator;
    mm::MazeMap map;
    mm::MazeMap truth;
    nav::MazeMissionController mission;
    uint32_t timestamp = 1000;
    int16_t yaw = 0;

    MissionFixture()
        : tracker(odometry, orientation),
          wallPerception(wallSensor),
          estimator(tracker, wallPerception),
          mission(map, storage, estimator) {}

    bool begin(const MazeConfig& config, bool addObstacle = false) {
        if (!truth.init(config.cols, config.rows)) return false;
        for (uint8_t x = 0; x < config.cols; ++x) {
            truth.setWall(x, 0, mm::Dir::North, true);
            truth.setWall(x, config.rows - 1, mm::Dir::South, true);
        }
        for (uint8_t y = 0; y < config.rows; ++y) {
            truth.setWall(0, y, mm::Dir::West, true);
            truth.setWall(config.cols - 1, y, mm::Dir::East, true);
        }
        if (addObstacle && config.cols > 2 && config.rows > 1) {
            // Obriga o Flood Fill a recalcular quando a parede for percebida.
            truth.setWall(1, 0, mm::Dir::East, true);
        }
        yaw = 0;
        orientation.setYawDegrees(yaw, timestamp);
        odometry.setAccumulatedDistanceMm(0, timestamp);
        return mission.begin(config, 0);
    }

    void enqueueCellSnapshot() {
        const mm::Cell* cell = truth.get(mission.x(), mission.y());
        const mm::Dir heading = mission.heading();
        if (cell == nullptr) return;

        const mm::Dir front = heading;
        const mm::Dir left = mm::MazeMap::relToAbs(heading, mm::Rel::Left);
        const mm::Dir right = mm::MazeMap::relToAbs(heading, mm::Rel::Right);
        for (int sample = 0; sample < 3; ++sample) {
            ++timestamp;
            wallSensor.enqueue(perception::Direction::Front,
                cell->hasWall(front)
                    ? perception::RawEcho{60, true, timestamp}
                    : perception::RawEcho{0, false, timestamp});
            wallSensor.enqueue(perception::Direction::Left,
                cell->hasWall(left)
                    ? perception::RawEcho{60, true, timestamp}
                    : perception::RawEcho{0, false, timestamp});
            wallSensor.enqueue(perception::Direction::Right,
                cell->hasWall(right)
                    ? perception::RawEcho{60, true, timestamp}
                    : perception::RawEcho{0, false, timestamp});
        }
        for (int sample = 0; sample < 3; ++sample) estimator.updateWalls();
    }

    nav::MissionUpdate observe() {
        enqueueCellSnapshot();
        return mission.observeCurrentCell();
    }

    nav::MissionUpdate completeMotion(nav::MissionUpdate action) {
        if (action.state == nav::MissionState::Failed ||
            action.action == nav::MissionAction::Brake) return action;

        if (action.action == nav::MissionAction::TurnLeft90 ||
            action.action == nav::MissionAction::TurnRight90 ||
            action.action == nav::MissionAction::TurnAround180) {
            int16_t delta = 0;
            if (action.action == nav::MissionAction::TurnLeft90) delta = -90;
            else if (action.action == nav::MissionAction::TurnRight90) delta = 90;
            else delta = -180;
            int value = static_cast<int>(yaw) + delta;
            while (value > 180) value -= 360;
            while (value < -180) value += 360;
            yaw = static_cast<int16_t>(value);
            timestamp += 250;
            orientation.setYawDegrees(yaw, timestamp);
            odometry.setAccumulatedDistanceMm(0, timestamp);
            estimator.updatePose();
            action = mission.onMotionFinished(true);
            if (action.action != nav::MissionAction::MoveForwardOneCell) return action;
        }

        const bool moveForward = action.action == nav::MissionAction::MoveForwardOneCell;
        const bool moveBackward = action.action == nav::MissionAction::MoveBackwardOneCell;
        if (!moveForward && !moveBackward) return action;
        timestamp += 300;
        const int16_t signedDistance = moveForward
            ? tracking::kCellDistanceThresholdMm
            : static_cast<int16_t>(-tracking::kCellDistanceThresholdMm);
        odometry.setAccumulatedDistanceMm(signedDistance, timestamp);
        estimator.updatePose();
        return mission.onMotionFinished(true);
    }
};

bool reachesGoal(MazeCorner corner, Direction initialHeading) {
    MissionFixture fixture;
    const MazeConfig config = makeMazeConfig(MazeFormat::Maze4x4, corner, initialHeading);
    if (!fixture.begin(config)) return false;

    for (uint16_t step = 0; step < 100; ++step) {
        if (fixture.mission.state() != nav::MissionState::AwaitingObservation) return false;
        nav::MissionUpdate action = fixture.observe();
        if (action.state == nav::MissionState::Completed) {
            mm::MazeMap restored;
            return fixture.mission.x() == config.goal.col &&
                   fixture.mission.y() == config.goal.row &&
                   fixture.storage.length > 0 && restored.load(fixture.storage) &&
                   restored.get(config.goal.col, config.goal.row)->visited;
        }
        if (action.state == nav::MissionState::Failed) return false;
        action = fixture.completeMotion(action);
        if (action.state == nav::MissionState::Failed) return false;
    }
    return false;
}

bool reachesGoalWithDetour() {
    MissionFixture fixture;
    const MazeConfig config = makeMazeConfig(
        MazeFormat::Maze8x4, MazeCorner::TopLeft, Direction::East);
    if (!fixture.begin(config, true)) return false;

    for (uint16_t step = 0; step < 150; ++step) {
        if (fixture.mission.state() != nav::MissionState::AwaitingObservation) return false;
        nav::MissionUpdate action = fixture.observe();
        if (action.state == nav::MissionState::Completed) {
            mm::MazeMap restored;
            return fixture.mission.x() == config.goal.col &&
                   fixture.mission.y() == config.goal.row &&
                   restored.load(fixture.storage) &&
                   restored.get(config.goal.col, config.goal.row)->visited;
        }
        if (action.state == nav::MissionState::Failed) return false;
        action = fixture.completeMotion(action);
        if (action.state == nav::MissionState::Failed) return false;
    }
    return false;
}

}  // namespace

void test_configura_origem_objetivo_nos_quatro_cantos(void) {
    TEST_ASSERT_TRUE(isValidMazeConfig(makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::East)));
    TEST_ASSERT_TRUE(isValidMazeConfig(makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopRight, Direction::West)));
    TEST_ASSERT_TRUE(isValidMazeConfig(makeMazeConfig(
        MazeFormat::Maze8x4, MazeCorner::BottomLeft, Direction::North)));
    const MazeConfig maze12x4 = makeMazeConfig(
        MazeFormat::Maze12x4, MazeCorner::BottomRight, Direction::North);
    TEST_ASSERT_TRUE(isValidMazeConfig(maze12x4));
    TEST_ASSERT_EQUAL_UINT8(12, maze12x4.cols);
    TEST_ASSERT_EQUAL_UINT8(4, maze12x4.rows);
    TEST_ASSERT_EQUAL_UINT8(11, maze12x4.start.col);
    TEST_ASSERT_EQUAL_UINT8(3, maze12x4.start.row);
    const MazeConfig maze8x4 = makeMazeConfig(
        MazeFormat::Maze8x4, MazeCorner::TopLeft, Direction::East);
    TEST_ASSERT_EQUAL_UINT8(8, maze8x4.cols);
    TEST_ASSERT_EQUAL_UINT8(4, maze8x4.rows);
}

void test_rejeita_objetivo_nao_oposto_ao_inicio(void) {
    MazeConfig config = makeMazeConfig(
        MazeFormat::Maze8x4, MazeCorner::TopRight, Direction::West);
    config.goal = {0, 3};
    TEST_ASSERT_FALSE(isValidMazeConfig(config));
}

void test_missao_usa_pose_estimada_e_persiste_mapa_final(void) {
    TEST_ASSERT_TRUE(reachesGoal(MazeCorner::TopLeft, Direction::East));
    TEST_ASSERT_TRUE(reachesGoal(MazeCorner::TopRight, Direction::West));
    TEST_ASSERT_TRUE(reachesGoal(MazeCorner::BottomLeft, Direction::North));
    TEST_ASSERT_TRUE(reachesGoal(MazeCorner::BottomRight, Direction::North));
}

void test_missao_recalcula_rota_ao_identificar_obstaculo_8x4(void) {
    TEST_ASSERT_TRUE(reachesGoalWithDetour());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_configura_origem_objetivo_nos_quatro_cantos);
    RUN_TEST(test_rejeita_objetivo_nao_oposto_ao_inicio);
    RUN_TEST(test_missao_usa_pose_estimada_e_persiste_mapa_final);
    RUN_TEST(test_missao_recalcula_rota_ao_identificar_obstaculo_8x4);
    return UNITY_END();
}
