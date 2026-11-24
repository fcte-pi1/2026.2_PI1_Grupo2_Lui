#include <unity.h>
#include "hal/SnapshotWallSensor.h"
#include "navigation/WallPerception.h"
using perception::Direction;
SonarSnapshot source;
uint32_t clockMs;
int captures;
SonarSnapshot capture(){++captures;return source;}
uint32_t clockNow(){return clockMs;}
void setUp(){source={};source.front={true,20,1160};source.left={true,3,174};source.right={true,4,232};source.ready=true;source.sampledAtMs=100;clockMs=100;captures=0;}
void tearDown(){}
void test_all_directions_use_one_coherent_hardware_frame(){
 SnapshotWallSensor adapter(capture,clockNow);
 TEST_ASSERT_EQUAL(200,adapter.read(Direction::Front).rawDistanceMm);
 source.left.distanceCm=30;
 TEST_ASSERT_EQUAL(30,adapter.read(Direction::Left).rawDistanceMm);
 TEST_ASSERT_EQUAL(40,adapter.read(Direction::Right).rawDistanceMm);
 TEST_ASSERT_EQUAL(1,captures);
 adapter.read(Direction::Front);
 TEST_ASSERT_EQUAL(300,adapter.read(Direction::Left).rawDistanceMm);
 TEST_ASSERT_EQUAL(2,captures);
}
void test_physical_no_echo_is_unknown_in_actual_wall_perception(){
 source.left={false,NAN,0};
 SnapshotWallSensor adapter(capture,clockNow);
 perception::WallPerception walls(adapter);walls.update();
 TEST_ASSERT_FALSE(walls.currentState().leftValid);
 TEST_ASSERT_FALSE(walls.currentState().wallLeft);
 TEST_ASSERT_TRUE(walls.currentState().frontValid);
 TEST_ASSERT_TRUE(walls.currentState().rightValid);
}
void test_too_close_pulse_is_a_touching_wall_not_unknown(){
 // Encostado na parede o sonar fica abaixo do alcance mínimo (teste no labirinto, 23/11).
 source.front={false,2.5f,80,true};
 SnapshotWallSensor adapter(capture,clockNow);
 perception::WallPerception walls(adapter);walls.update();
 TEST_ASSERT_TRUE(walls.currentState().frontValid);
 TEST_ASSERT_TRUE(walls.currentState().wallFront);
 TEST_ASSERT_EQUAL(20,walls.currentState().distFrontMm);
}
void test_echo_lost_right_after_short_reading_is_still_a_wall(){
 SnapshotWallSensor adapter(capture,clockNow);
 source.right={true,2.4f,140};
 TEST_ASSERT_TRUE(adapter.read(Direction::Front).valid);
 adapter.read(Direction::Left);
 TEST_ASSERT_EQUAL(24,adapter.read(Direction::Right).rawDistanceMm);
 source.right={false,NAN,0};source.sampledAtMs=clockMs=300;
 adapter.read(Direction::Front);adapter.read(Direction::Left);
 const auto held=adapter.read(Direction::Right);
 TEST_ASSERT_TRUE(held.valid);TEST_ASSERT_EQUAL(24,held.rawDistanceMm);
 // Depois de 1 s sem nova leitura curta, volta a ser desconhecido.
 source.sampledAtMs=clockMs=1301;
 adapter.read(Direction::Front);adapter.read(Direction::Left);
 TEST_ASSERT_FALSE(adapter.read(Direction::Right).valid);
}
void test_stale_or_unready_frame_is_never_an_observed_opening(){
 clockMs=source.sampledAtMs+SonarSampling::MAX_SAMPLE_AGE_MS+1;
 SnapshotWallSensor adapter(capture,clockNow);
 TEST_ASSERT_FALSE(adapter.read(Direction::Front).valid);
 source.ready=false;clockMs=100;
 TEST_ASSERT_FALSE(adapter.read(Direction::Front).valid);
}
void test_frame_freshness_handles_clock_wrap(){
 source.sampledAtMs=UINT32_MAX-50;clockMs=25;
 SnapshotWallSensor adapter(capture,clockNow);
 TEST_ASSERT_TRUE(adapter.read(Direction::Front).valid);
 clockMs=source.sampledAtMs+SonarSampling::MAX_SAMPLE_AGE_MS+1;
 TEST_ASSERT_FALSE(adapter.read(Direction::Front).valid);
}
void test_cached_frame_becoming_stale_invalidates_published_distances(){
 SnapshotWallSensor adapter(capture,clockNow);
 perception::WallPerception walls(adapter);walls.update();
 TEST_ASSERT_TRUE(walls.currentState().leftValid);
 TEST_ASSERT_EQUAL(30,walls.currentState().distLeftMm);
 clockMs=source.sampledAtMs+SonarSampling::MAX_SAMPLE_AGE_MS+1;
 walls.update();
 TEST_ASSERT_FALSE(walls.currentState().frontValid);
 TEST_ASSERT_FALSE(walls.currentState().leftValid);
 TEST_ASSERT_FALSE(walls.currentState().rightValid);
 TEST_ASSERT_EQUAL(0,walls.currentState().distLeftMm);
 TEST_ASSERT_FALSE(walls.currentState().wallLeft);
 // Polling the same expired frame is not five new acquisition failures.
 for(int i=0;i<10;++i) walls.update();
 TEST_ASSERT_FALSE(walls.hasFault(Direction::Left));
 source.sampledAtMs=clockMs;
 walls.update();
 TEST_ASSERT_TRUE(walls.currentState().leftValid);
 TEST_ASSERT_EQUAL(30,walls.currentState().distLeftMm);
}
void test_first_wall_after_sensor_failure_does_not_reuse_old_open_readings(){
 SnapshotWallSensor adapter(capture,clockNow);
 perception::WallPerception walls(adapter);
 source.front={true,100,5800};
 for(int i=0;i<3;++i){source.sampledAtMs=clockMs++;walls.update();}
 TEST_ASSERT_FALSE(walls.currentState().wallFront);
 source.front={false,NAN,0};source.sampledAtMs=clockMs++;walls.update();
 TEST_ASSERT_FALSE(walls.currentState().frontValid);
 source.front={true,3,174};source.sampledAtMs=clockMs++;walls.update();
 TEST_ASSERT_TRUE(walls.currentState().frontValid);
 TEST_ASSERT_TRUE(walls.currentState().wallFront);
 TEST_ASSERT_EQUAL(30,walls.currentState().distFrontMm);
 // Lateral filters are independent of the failed front channel.
 TEST_ASSERT_TRUE(walls.currentState().leftValid);
 TEST_ASSERT_EQUAL(30,walls.currentState().distLeftMm);
}
void test_full_sequential_cycle_is_not_stale_before_the_next_publication(){
 SnapshotWallSensor adapter(capture,clockNow);
 const uint32_t oldest=source.sampledAtMs;
 // At 20ms per echo and 25ms quiet intervals, age reaches 245ms.
 clockMs=oldest+245;
 TEST_ASSERT_TRUE(adapter.read(Direction::Front).valid);
 clockMs=oldest+SonarSampling::MAX_SAMPLE_AGE_MS+1;
 TEST_ASSERT_FALSE(adapter.read(Direction::Front).valid);
}
void test_filtered_snapshot_keeps_single_missing_side_but_expires_real_evidence(){
 source.hasFilters=true;source.filteredFront={true,20,100};source.filteredLeft={true,3,100};source.filteredRight={true,4,100};
 source.left={false,NAN,0};SnapshotWallSensor adapter(capture,clockNow);
 const auto front=adapter.read(Direction::Front);const auto left=adapter.read(Direction::Left);
 TEST_ASSERT_TRUE(front.valid);TEST_ASSERT_TRUE(left.valid);TEST_ASSERT_TRUE(left.preFiltered);
 TEST_ASSERT_EQUAL(30,left.rawDistanceMm);TEST_ASSERT_EQUAL(0,source.left.pulseUs);
 adapter.read(Direction::Right);clockMs=100+SonarSampling::MAX_SAMPLE_AGE_MS+1;
 TEST_ASSERT_FALSE(adapter.read(Direction::Front).valid);
 TEST_ASSERT_FALSE(adapter.read(Direction::Left).valid);
}
void test_hardware_median_is_not_averaged_again_by_wall_perception(){
 source.hasFilters=true;source.filteredFront={true,12,100};source.filteredLeft={true,3,100};source.filteredRight={true,4,100};
 SnapshotWallSensor adapter(capture,clockNow);perception::WallPerception walls(adapter);walls.update();
 TEST_ASSERT_EQUAL(120,walls.currentState().distFrontMm);
 source.sampledAtMs=clockMs=200;source.filteredFront={true,3,200};source.filteredLeft.lastValidAtMs=source.filteredRight.lastValidAtMs=200;
 walls.update();TEST_ASSERT_EQUAL(30,walls.currentState().distFrontMm);TEST_ASSERT_TRUE(walls.currentState().wallFront);
}
int main(){UNITY_BEGIN();
 RUN_TEST(test_filtered_snapshot_keeps_single_missing_side_but_expires_real_evidence);
 RUN_TEST(test_hardware_median_is_not_averaged_again_by_wall_perception);
 RUN_TEST(test_full_sequential_cycle_is_not_stale_before_the_next_publication);
 RUN_TEST(test_cached_frame_becoming_stale_invalidates_published_distances);
 RUN_TEST(test_first_wall_after_sensor_failure_does_not_reuse_old_open_readings);
 RUN_TEST(test_all_directions_use_one_coherent_hardware_frame);
 RUN_TEST(test_physical_no_echo_is_unknown_in_actual_wall_perception);
 RUN_TEST(test_too_close_pulse_is_a_touching_wall_not_unknown);
 RUN_TEST(test_echo_lost_right_after_short_reading_is_still_a_wall);
 RUN_TEST(test_stale_or_unready_frame_is_never_an_observed_opening);
 RUN_TEST(test_frame_freshness_handles_clock_wrap);
 return UNITY_END();}
