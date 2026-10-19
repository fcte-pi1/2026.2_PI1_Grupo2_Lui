#include <unity.h>

#include "../test_position_tracker/mock_odometry_reader.h"
#include "../test_position_tracker/mock_orientation_sensor.h"
#include "../test_wall_perception/mock_wall_sensor.h"
#include "navigation/RobotStateEstimator.h"

using perception::Direction;
using perception::RawEcho;
using perception::WallPerception;
using perception::test::MockWallSensor;
using state_estimation::RobotStateEstimator;
using state_estimation::WallKnowledge;
using tracking::GridPosition;
using tracking::Orientation;
using tracking::PositionTracker;
using tracking::TrackingState;
using tracking::test::MockOdometryReader;
using tracking::test::MockOrientationSensor;

void setUp(void) {}
void tearDown(void) {}

static void test_relative_walls_are_projected_to_absolute_directions(void) {
    MockOdometryReader odometry;
    MockOrientationSensor orientation;
    MockWallSensor wallSensor;
    const TrackingState initialPose{GridPosition{3, 2}, Orientation::East, 0};
    PositionTracker tracker(odometry, orientation, initialPose, 0);
    WallPerception perception(wallSensor);
    RobotStateEstimator estimator(tracker, perception);

    wallSensor.enqueue(Direction::Front, RawEcho{50, true, 123});
    wallSensor.enqueue(Direction::Left, RawEcho{200, true, 123});
    wallSensor.enqueue(Direction::Right, RawEcho{70, true, 123});
    odometry.setAccumulatedDistanceMm(0, 123);
    orientation.setYawDegrees(0, 123);

    estimator.update();
    const auto& state = estimator.currentState();

    TEST_ASSERT_TRUE(state.initialized);
    TEST_ASSERT_EQUAL_INT16(3, state.pose.position.row);
    TEST_ASSERT_EQUAL_INT16(2, state.pose.position.col);
    TEST_ASSERT_EQUAL(static_cast<int>(Orientation::East), static_cast<int>(state.pose.orientation));

    // Voltado a Leste: frente=Leste, esquerda=Norte e direita=Sul.
    TEST_ASSERT_EQUAL(static_cast<int>(WallKnowledge::Present),
                      static_cast<int>(state.wallAt(Orientation::East).knowledge));
    TEST_ASSERT_EQUAL_UINT16(50, state.wallAt(Orientation::East).distanceMm);
    TEST_ASSERT_TRUE(state.wallAt(Orientation::East).distanceValid);

    TEST_ASSERT_EQUAL(static_cast<int>(WallKnowledge::Open),
                      static_cast<int>(state.wallAt(Orientation::North).knowledge));
    TEST_ASSERT_EQUAL_UINT16(200, state.wallAt(Orientation::North).distanceMm);

    TEST_ASSERT_EQUAL(static_cast<int>(WallKnowledge::Present),
                      static_cast<int>(state.wallAt(Orientation::South).knowledge));
    TEST_ASSERT_EQUAL_UINT16(70, state.wallAt(Orientation::South).distanceMm);

    // A HU-01 só define sensores frontais e laterais: Oeste permanece desconhecido.
    TEST_ASSERT_EQUAL(static_cast<int>(WallKnowledge::Unknown),
                      static_cast<int>(state.wallAt(Orientation::West).knowledge));
    TEST_ASSERT_FALSE(state.wallAt(Orientation::West).distanceValid);
    TEST_ASSERT_EQUAL_UINT32(123, state.sensorTimestampMs);
}

static void test_invalid_reading_is_unknown_but_no_echo_is_open(void) {
    MockOdometryReader odometry;
    MockOrientationSensor orientation;
    MockWallSensor wallSensor;
    PositionTracker tracker(odometry, orientation);
    WallPerception perception(wallSensor);
    RobotStateEstimator estimator(tracker, perception);

    wallSensor.enqueue(Direction::Front, RawEcho{0, true, 50});   // eco impossível
    wallSensor.enqueue(Direction::Left, RawEcho{0, false, 50});   // ausência de eco
    wallSensor.enqueue(Direction::Right, RawEcho{60, true, 50});
    odometry.setAccumulatedDistanceMm(0, 50);
    orientation.setYawDegrees(0, 50);

    estimator.update();
    const auto& state = estimator.currentState();

    TEST_ASSERT_EQUAL(static_cast<int>(WallKnowledge::Unknown),
                      static_cast<int>(state.wallAt(Orientation::North).knowledge));
    TEST_ASSERT_FALSE(state.wallAt(Orientation::North).distanceValid);
    TEST_ASSERT_EQUAL(static_cast<int>(WallKnowledge::Open),
                      static_cast<int>(state.wallAt(Orientation::West).knowledge));
    TEST_ASSERT_TRUE(state.wallAt(Orientation::West).distanceValid);
    TEST_ASSERT_EQUAL(static_cast<int>(WallKnowledge::Present),
                      static_cast<int>(state.wallAt(Orientation::East).knowledge));
    TEST_ASSERT_EQUAL(static_cast<int>(WallKnowledge::Unknown),
                      static_cast<int>(state.wallAt(Orientation::South).knowledge));
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_relative_walls_are_projected_to_absolute_directions);
    RUN_TEST(test_invalid_reading_is_unknown_but_no_echo_is_open);
    return UNITY_END();
}
