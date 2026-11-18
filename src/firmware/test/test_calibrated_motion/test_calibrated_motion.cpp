#include <unity.h>
#include <cmath>
#include <cstdint>
#include "control/motion_controller.hpp"
#include "navigation/GuidedRoute.h"
#include "config/maze_geometry.hpp"
#include "config/sensor_calibration.hpp"
#include "hal/ultrasonic_us100.hpp"
#include "control/corridor_feedback_filter.hpp"
#include "control/start_delay.hpp"
#include "control/front_echo_recovery.hpp"
#include "control/front_approach_policy.hpp"
#include "control/cell_observation_gate.hpp"
#include "control/front_proximity_guard.hpp"
#include "hal/sonar_median3.hpp"
#include "hal/sonar_snapshot.hpp"

uint32_t fakeNow=0;
uint32_t fakeEchoPulse=0;
int fakePwm[64]={}, fakeDigital[64]={};
TwoWire Wire;
int32_t counts[64]={};
float yaw=0,gyro=0;
bool encoderValid=true,imuValid=true;
bool EncoderN20::begin(uint8_t a,uint8_t,bool,uint32_t){_unit=reinterpret_cast<void*>(static_cast<uintptr_t>(a+1));_initialized=true;return true;}
bool EncoderN20::readCount(int32_t& c)const{c=counts[reinterpret_cast<uintptr_t>(_unit)-1];return encoderValid;}
bool EncoderN20::isInitialized()const{return _initialized;}
bool EncoderN20::resetCount(){counts[reinterpret_cast<uintptr_t>(_unit)-1]=0;return true;}
bool IMUMPU6500::begin(TwoWire&,uint8_t){_initialized=true;return true;}
bool IMUMPU6500::isInitialized()const{return _initialized;}
bool IMUMPU6500::read(MPU6500Reading& r){r={};r.yawDegrees=yaw;r.gyroZdps=gyro;_yawDegrees=yaw;return imuValid;}

struct Fixture {
 MotorDRV8833 motors;
 EncoderN20 left,right;
 IMUMPU6500 imu;
 Odometry odo;
 PIDController lp{1.5f,5,0,-80,100},rp{1.5f,5,0,-80,100};
 MotionController motion{motors,left,right,imu,odo,lp,rp};
 MotionControllerConfig cfg;
 Fixture(){fakeNow=0; yaw=gyro=0; encoderValid=imuValid=true; for(int i=0;i<64;++i){counts[i]=fakePwm[i]=fakeDigital[i]=0;}
  motors.begin(16,17,18,19,5);left.begin(32,33);right.begin(25,26);imu.begin();
  TEST_ASSERT_TRUE(motion.begin(cfg,{0,0,0}));}
 void arm(){motors.setEnabled(true);}
 void tick(int32_t l,int32_t r,float y,float g=0,uint32_t ms=5){counts[32]=l;counts[25]=r;yaw=y;gyro=g;fakeNow+=ms;motion.update(ms/1000.0f);}
 bool braking(){return fakePwm[16]==255 && fakePwm[18]==255 && fakeDigital[17] && fakeDigital[19];}
 void settle(float y){for(int n=0;n<210 && motion.getStatus()==MotionStatus::Running;++n)tick(counts[32],counts[25],y);}
};
void setUp(){}
void tearDown(){}
void test_motor_mode_matches_bench_and_disabled_boot(){
 Fixture f;TEST_ASSERT_FALSE(f.motors.isEnabled());TEST_ASSERT_EQUAL(LOW,fakeDigital[5]);
 f.motors.setSpeeds(150,150);TEST_ASSERT_EQUAL(0,fakePwm[16]);
 f.arm();f.motors.setSpeeds(-100,150);TEST_ASSERT_EQUAL(155,fakePwm[16]);TEST_ASSERT_EQUAL(HIGH,fakeDigital[17]);
 TEST_ASSERT_EQUAL(150,fakePwm[18]);TEST_ASSERT_EQUAL(LOW,fakeDigital[19]);
 f.motors.brake();TEST_ASSERT_TRUE(f.braking());f.motors.setEnabled(false);TEST_ASSERT_EQUAL(LOW,fakeDigital[5]);}
void test_unarmed_move_rejected(){Fixture f;TEST_ASSERT_FALSE(f.motion.startMoveForwardOneCell());TEST_ASSERT_EQUAL((int)MotionFailureReason::MotorsDisabled,(int)f.motion.getFailureReason());}
void test_linear_completes_only_after_brake_and_quiet(){
 Fixture f;f.arm();TEST_ASSERT_TRUE(f.motion.startMoveForwardOneCell());
 for(int n=1;n<700 && !f.braking();++n){float mm=n*0.3f;f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);}
 TEST_ASSERT_TRUE(f.braking());TEST_ASSERT_EQUAL((int)MotionStatus::Running,(int)f.motion.getStatus());
 TEST_ASSERT_FALSE(f.motion.startTurnRight90());
 const uint32_t started=fakeNow;f.settle(0);
 TEST_ASSERT_TRUE(fakeNow-started>=300);TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
 TEST_ASSERT_FLOAT_WITHIN(5,180,f.motion.getDistanceTravelledMm());TEST_ASSERT_FALSE(f.motors.isEnabled());}
void test_heading_correction_does_not_brake_right_for_encoder_lead(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();
 for(int n=1;n<=300;++n){float mm=n*0.3f;f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*1.08f*f.cfg.rightCountsPerMm),-1);}
 TEST_ASSERT_EQUAL((int)MotionStatus::Running,(int)f.motion.getStatus());
 TEST_ASSERT_FALSE(fakePwm[18]==255 && fakeDigital[19]);TEST_ASSERT_TRUE(fakePwm[18]<=220);}
void test_turn_180_crosses_yaw_wrap(){
 Fixture f;yaw=-170;f.arm();TEST_ASSERT_TRUE(f.motion.startTurnAround180());
 float finalYaw=yaw;
 for(int n=1;n<=360 && !f.braking();++n){float a=n*0.5f;float y=-170-a;while(y< -180)y+=360;finalYaw=y;
  float arc=a*3.14159265f/180*60;f.tick(lroundf(arc*f.cfg.leftCountsPerMm),-lroundf(arc*f.cfg.rightCountsPerMm),y,a<160?-55:-10,10);}
 TEST_ASSERT_TRUE(f.braking());f.settle(finalYaw);TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
 TEST_ASSERT_FLOAT_WITHIN(1.2f,-180,f.motion.getTurnAngleDeg());}
void test_turn_left_crosses_yaw_wrap(){
 Fixture f;yaw=170;f.arm();TEST_ASSERT_TRUE(f.motion.startTurnLeft90());float finalYaw=yaw;
 for(int n=1;n<=180 && !f.braking();++n){float a=n*0.5f;float y=170+a;while(y>=180)y-=360;finalYaw=y;
  float arc=a*3.14159265f/180*60;f.tick(-lroundf(arc*f.cfg.leftCountsPerMm),lroundf(arc*f.cfg.rightCountsPerMm),y,a<70?55:10,10);}
 TEST_ASSERT_TRUE(f.braking());f.settle(finalYaw);TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
 TEST_ASSERT_FLOAT_WITHIN(1.2f,90,f.motion.getTurnAngleDeg());}
void test_stalled_wheels_fail_after_braking(){Fixture f;f.arm();f.motion.startMoveForwardOneCell();for(int n=0;n<230;++n)f.tick(0,0,0);
 TEST_ASSERT_EQUAL((int)MotionStatus::Failed,(int)f.motion.getStatus());TEST_ASSERT_EQUAL((int)MotionFailureReason::LeftWheelStalled,(int)f.motion.getFailureReason());TEST_ASSERT_FALSE(f.motors.isEnabled());}
void test_imu_failure_disables_immediately(){Fixture f;f.arm();f.motion.startMoveForwardOneCell();imuValid=false;f.tick(0,0,0);
 TEST_ASSERT_EQUAL((int)MotionFailureReason::ImuReadFailed,(int)f.motion.getFailureReason());TEST_ASSERT_FALSE(f.motors.isEnabled());}
void test_excess_heading_fails_not_success(){Fixture f;f.arm();f.motion.startMoveForwardOneCell();f.tick(10,10,16);TEST_ASSERT_TRUE(f.braking());f.settle(16);
 TEST_ASSERT_EQUAL((int)MotionStatus::Failed,(int)f.motion.getStatus());TEST_ASSERT_EQUAL((int)MotionFailureReason::SafetyLimit,(int)f.motion.getFailureReason());}
void test_braking_that_never_settles_fails(){Fixture f;f.arm();f.motion.startMoveForwardOneCell();f.tick(0,0,16,30);
 for(int n=0;n<210;++n)f.tick(0,0,16,30);
 TEST_ASSERT_EQUAL((int)MotionFailureReason::BrakeNotSettled,(int)f.motion.getFailureReason());}
void test_reverse_signed_motion_keeps_heading_and_brakes(){
 Fixture f;f.arm();TEST_ASSERT_TRUE(f.motion.startMoveBackwardOneCell());
 for(int n=1;n<700 && !f.braking();++n){float mm=n*0.3f;f.tick(-lroundf(mm*f.cfg.leftCountsPerMm),-lroundf(mm*f.cfg.rightCountsPerMm),0);}
 TEST_ASSERT_TRUE(f.braking());f.settle(0);TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
 TEST_ASSERT_FLOAT_WITHIN(5,-180,f.motion.getDistanceTravelledMm());}
void test_nonfinite_imu_never_generates_motor_output(){Fixture f;f.arm();f.motion.startMoveForwardOneCell();f.tick(0,0,NAN);
 TEST_ASSERT_EQUAL((int)MotionFailureReason::ImuReadFailed,(int)f.motion.getFailureReason());TEST_ASSERT_FALSE(f.motors.isEnabled());}
void test_guided_route_is_two_forward_approach_left_forward(){
 nav::GuidedRoute route;
 const nav::GuidedAction expected[]={nav::GuidedAction::ForwardCell,nav::GuidedAction::ForwardCell,
 nav::GuidedAction::ApproachFrontWall,nav::GuidedAction::TurnLeft90,nav::GuidedAction::ForwardCell};
 for(int i=0;i<5;++i){TEST_ASSERT_EQUAL(i,route.step());TEST_ASSERT_EQUAL((int)expected[i],(int)route.action());route.finishMovement(true);}
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::Complete,(int)route.action());route.finishMovement(true);TEST_ASSERT_EQUAL(5,route.step());}
void test_guided_route_cannot_advance_after_failure(){nav::GuidedRoute route;route.finishMovement(true);route.finishMovement(false);
 route.finishMovement(true);TEST_ASSERT_EQUAL(1,route.step());TEST_ASSERT_EQUAL((int)nav::GuidedAction::Failed,(int)route.action());
 route.reset();TEST_ASSERT_EQUAL(0,route.step());TEST_ASSERT_EQUAL((int)nav::GuidedAction::ForwardCell,(int)route.action());}
void test_corner_continuation_does_not_repeat_forward_cells(){nav::GuidedRoute route;route.resetAtCorner();
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::ApproachFrontWall,(int)route.action());route.finishMovement(true);
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::TurnLeft90,(int)route.action());route.finishMovement(true);
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::ForwardCell,(int)route.action());route.finishMovement(true);
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::Complete,(int)route.action());}
void test_confirmed_geometry_is_17_free_plus_1_wall(){
 TEST_ASSERT_EQUAL(170,MazeSettings::CLEAR_CELL_MM);TEST_ASSERT_EQUAL(10,MazeSettings::WALL_THICKNESS_MM);
 TEST_ASSERT_EQUAL(180,MazeSettings::CELL_SIZE_MM);}
void test_wall_centering_steers_away_from_closer_left_wall(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();f.motion.setWallAlignmentFeedback(25,45,true);
 for(int n=1;n<=100;++n){float mm=n*0.3f;f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);}
 TEST_ASSERT_TRUE(fakePwm[16]>fakePwm[18]);TEST_ASSERT_FALSE(f.braking());}
void test_opening_does_not_enter_wall_alignment_loop(){
 Fixture a;a.arm();a.motion.startMoveForwardOneCell();a.motion.setWallAlignmentFeedback(370,30,true);a.tick(1,1,0);
 int leftPwm=fakePwm[16],rightPwm=fakePwm[18];
 Fixture b;b.arm();b.motion.startMoveForwardOneCell();b.tick(1,1,0);
 TEST_ASSERT_EQUAL(leftPwm,fakePwm[16]);TEST_ASSERT_EQUAL(rightPwm,fakePwm[18]);}
void test_front_approach_has_low_speed_and_can_brake_on_sensor_target(){
 Fixture f;f.motion.setFrontApproachMode(true);f.arm();TEST_ASSERT_TRUE(f.motion.startMoveLinear(25));
 for(int n=1;n<100;++n)f.tick(n/5,n/5,0);
 f.motion.brakeAtFrontTarget();TEST_ASSERT_TRUE(f.braking());f.settle(0);
 TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());TEST_ASSERT_FALSE(f.motors.isEnabled());}
void test_ordinary_motion_cannot_finish_via_front_target_command(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();f.motion.brakeAtFrontTarget();
 TEST_ASSERT_FALSE(f.braking());TEST_ASSERT_EQUAL((int)MotionStatus::Running,(int)f.motion.getStatus());}
void test_pre_corner_distance_respects_wall_and_cell_budget(){
 TEST_ASSERT_FLOAT_WITHIN(0.01f,180,nav::guidedPreCornerMoveMm(36));
 TEST_ASSERT_FLOAT_WITHIN(0.01f,100,nav::guidedPreCornerMoveMm(18));
 TEST_ASSERT_FLOAT_WITHIN(0.01f,0,nav::guidedPreCornerMoveMm(7));
 TEST_ASSERT_FLOAT_WITHIN(0.01f,0,nav::guidedPreCornerMoveMm(NAN));}
void test_sonar_calibrates_before_minimum_range_check(){
 UltrasonicUS100 sensor;sensor.begin(4,35);sensor.setDistanceOffsetCm(SensorCalibration::RIGHT_OFFSET_CM);
 fakeEchoPulse=115;auto value=sensor.measure();TEST_ASSERT_TRUE(value.valid);TEST_ASSERT_EQUAL(115,value.pulseUs);
 TEST_ASSERT_FLOAT_WITHIN(0.01f,2.633f,value.distanceCm);}
void test_sonar_no_echo_stays_invalid_with_positive_offset(){
 UltrasonicUS100 sensor;sensor.begin(4,35);sensor.setDistanceOffsetCm(0.82f);fakeEchoPulse=0;
 auto value=sensor.measure();TEST_ASSERT_FALSE(value.valid);TEST_ASSERT_TRUE(std::isnan(value.distanceCm));TEST_ASSERT_EQUAL(0,value.pulseUs);}
void test_sonar_preserves_too_close_echo_without_claiming_accuracy(){
 UltrasonicUS100 sensor;sensor.begin(4,35);fakeEchoPulse=80;auto value=sensor.measure();
 TEST_ASSERT_FALSE(value.valid);TEST_ASSERT_TRUE(value.tooClose);TEST_ASSERT_EQUAL(80,value.pulseUs);TEST_ASSERT_TRUE(std::isfinite(value.distanceCm));}
void test_sonar_rejects_invalid_calibration(){UltrasonicUS100 sensor;
 TEST_ASSERT_FALSE(sensor.setDistanceOffsetCm(NAN));TEST_ASSERT_FALSE(sensor.setDistanceOffsetCm(3));TEST_ASSERT_FLOAT_WITHIN(0.001f,0,sensor.distanceOffsetCm());}
void test_sonar_provisional_offsets_fit_reference_band(){
 const float nearRaw[]={2.310f,2.638f,2.310f},farRaw[]={9.052f,9.440f,9.397f};
 const float offsets[]={SensorCalibration::FRONT_OFFSET_CM,SensorCalibration::LEFT_OFFSET_CM,SensorCalibration::RIGHT_OFFSET_CM};
 for(int i=0;i<3;++i){TEST_ASSERT_FLOAT_WITHIN(0.5f,3,nearRaw[i]+offsets[i]);TEST_ASSERT_FLOAT_WITHIN(0.5f,10,farRaw[i]+offsets[i]);}}
void test_calibration_cannot_hide_very_short_raw_echo(){
 UltrasonicUS100 sensor;sensor.begin(4,35);sensor.setDistanceOffsetCm(0.82f);fakeEchoPulse=87;
 auto value=sensor.measure();TEST_ASSERT_FALSE(value.valid);TEST_ASSERT_TRUE(value.tooClose);TEST_ASSERT_EQUAL(87,value.pulseUs);}

void test_lateral_median_requires_three_new_frames_and_rejects_single_outlier() {
 CorridorFeedbackFilter3 f;
 TEST_ASSERT_FALSE(f.update(30,40,true,10,10).valid);
 for (int i=0;i<100;++i) TEST_ASSERT_FALSE(f.update(70,40,true,10,10).changed);
 TEST_ASSERT_FALSE(f.update(70,40,true,85,85).valid);
 const auto result=f.update(31,41,true,160,160);
 TEST_ASSERT_TRUE(result.changed);TEST_ASSERT_TRUE(result.valid);
 TEST_ASSERT_FLOAT_WITHIN(0.01f,31,result.leftMm);
 TEST_ASSERT_FLOAT_WITHIN(0.01f,40,result.rightMm);
}
void test_lateral_opening_disables_immediately_and_rebuilds_window() {
 CorridorFeedbackFilter3 f;
 for(uint32_t t=10;t<=160;t+=75) f.update(30,40,true,t,t);
 TEST_ASSERT_FALSE(f.update(300,40,true,235,235).valid);
 TEST_ASSERT_FALSE(f.update(30,40,true,310,310).valid);
 TEST_ASSERT_FALSE(f.update(30,40,true,385,385).valid);
 TEST_ASSERT_TRUE(f.update(30,40,true,460,460).valid);
}
void test_lateral_invalid_echo_or_nan_clears_history() {
 CorridorFeedbackFilter3 f;
 for(uint32_t t=10;t<=160;t+=75) f.update(30,40,true,t,t);
 TEST_ASSERT_FALSE(f.update(30,40,false,235,235).valid);
 TEST_ASSERT_FALSE(f.update(30,40,true,310,310).valid);
 TEST_ASSERT_FALSE(f.update(NAN,40,true,385,385).valid);
 TEST_ASSERT_FALSE(f.update(30,40,true,460,460).valid);
 TEST_ASSERT_FALSE(f.update(30,40,true,535,535).valid);
 TEST_ASSERT_TRUE(f.update(30,40,true,610,610).valid);
}
void test_lateral_stale_frame_disables_even_if_timestamp_repeated() {
 CorridorFeedbackFilter3 f;
 for(uint32_t t=10;t<=160;t+=75) f.update(30,40,true,t,t);
 const auto stale=f.update(30,40,true,160,160+SonarSampling::MAX_SAMPLE_AGE_MS+1);
 TEST_ASSERT_TRUE(stale.changed);TEST_ASSERT_FALSE(stale.valid);
 TEST_ASSERT_FALSE(f.update(30,40,true,500,500).valid);
 TEST_ASSERT_FALSE(f.update(30,40,true,575,575).valid);
 TEST_ASSERT_TRUE(f.update(30,40,true,650,650).valid);
}
void test_lateral_pause_gap_and_explicit_reset_need_fresh_window() {
 CorridorFeedbackFilter3 f;
 for(uint32_t t=10;t<=160;t+=75) f.update(30,40,true,t,t);
 TEST_ASSERT_FALSE(f.update(30,40,true,1000,1000).valid);
 TEST_ASSERT_FALSE(f.update(30,40,true,1075,1075).valid);
 TEST_ASSERT_TRUE(f.update(30,40,true,1150,1150).valid);
 f.reset();
 TEST_ASSERT_FALSE(f.update(30,40,true,1225,1225).valid);
}
void test_lateral_median_handles_millis_wrap() {
 CorridorFeedbackFilter3 f;
 const uint32_t first=UINT32_MAX-100;
 TEST_ASSERT_FALSE(f.update(30,40,true,first,first+10).valid);
 TEST_ASSERT_FALSE(f.update(31,41,true,first+75,first+85).valid);
 TEST_ASSERT_TRUE(f.update(32,42,true,first+150,first+160).valid);
 TEST_ASSERT_FALSE(f.update(32,42,true,first+150,first+150+SonarSampling::MAX_SAMPLE_AGE_MS+1).valid);
}
void test_lateral_small_difference_does_not_change_wheel_command() {
 Fixture a;a.arm();a.motion.startMoveForwardOneCell();
 a.motion.setWallAlignmentFeedback(30,32,true);a.tick(1,1,0);
 const int left=fakePwm[16],right=fakePwm[18];
 Fixture b;b.arm();b.motion.startMoveForwardOneCell();b.tick(1,1,0);
 TEST_ASSERT_EQUAL(left,fakePwm[16]);TEST_ASSERT_EQUAL(right,fakePwm[18]);
}


void test_start_delay_is_cancellable_and_consumed_once() {
 StartDelay delay;delay.arm(100,5000);
 TEST_ASSERT_TRUE(delay.pending());TEST_ASSERT_FALSE(delay.consumeIfDue(5099));
 TEST_ASSERT_TRUE(delay.consumeIfDue(5100));TEST_ASSERT_FALSE(delay.consumeIfDue(5101));
 delay.arm(6000,5000);delay.cancel();TEST_ASSERT_FALSE(delay.consumeIfDue(12000));
}
void test_start_delay_handles_millis_wrap() {
 StartDelay delay;const uint32_t start=UINT32_MAX-2000;delay.arm(start,5000);
 TEST_ASSERT_FALSE(delay.consumeIfDue(start+4999));TEST_ASSERT_TRUE(delay.consumeIfDue(start+5000));
}

void test_echo_recovery_requires_two_new_valid_frames_and_brake_settling(){
 FrontEchoRecovery r;using D=FrontEchoRecovery::Decision;
 TEST_ASSERT_EQUAL((int)D::Proceed,(int)r.update(true,1,1,false));
 TEST_ASSERT_EQUAL((int)D::Hold,(int)r.update(false,100,100,false));
 TEST_ASSERT_EQUAL((int)D::Hold,(int)r.update(true,180,180,true));
 for(int i=0;i<100;++i)TEST_ASSERT_EQUAL((int)D::Hold,(int)r.update(true,180,300,true));
 TEST_ASSERT_EQUAL((int)D::Hold,(int)r.update(true,310,310,false));
 TEST_ASSERT_EQUAL((int)D::Proceed,(int)r.update(true,310,320,true));
 TEST_ASSERT_EQUAL(1,r.episodes());
}
void test_echo_recovery_timeout_is_latched_and_cannot_resume_itself(){
 FrontEchoRecovery r;using D=FrontEchoRecovery::Decision;
 r.update(false,10,10,false);r.update(true,20,20,true);
 TEST_ASSERT_EQUAL((int)D::Failed,(int)r.update(true,1210,1210,true));
 TEST_ASSERT_EQUAL((int)D::Failed,(int)r.update(true,1300,1300,true));
 r.reset();TEST_ASSERT_EQUAL((int)D::Proceed,(int)r.update(true,1300,1300,true));
}
void test_echo_recovery_rejects_more_than_three_episodes_in_one_segment(){
 FrontEchoRecovery r;using D=FrontEchoRecovery::Decision;
 for(uint32_t i=0;i<3;++i){const uint32_t t=1000*i;
  TEST_ASSERT_EQUAL((int)D::Hold,(int)r.update(false,t,t,false));
  r.update(true,t+100,t+100,true);
  TEST_ASSERT_EQUAL((int)D::Proceed,(int)r.update(true,t+210,t+210,true));
 }
 TEST_ASSERT_EQUAL((int)D::Failed,(int)r.update(false,4000,4000,false));
}
void test_echo_recovery_good_sequence_resets_on_another_invalid_frame(){
 FrontEchoRecovery r;using D=FrontEchoRecovery::Decision;
 r.update(false,0,0,false);r.update(true,100,100,true);r.update(false,200,200,true);
 TEST_ASSERT_EQUAL((int)D::Hold,(int)r.update(true,300,300,true));
 TEST_ASSERT_EQUAL((int)D::Proceed,(int)r.update(true,400,400,true));
}
void test_echo_recovery_handles_millis_wrap(){
 FrontEchoRecovery r;using D=FrontEchoRecovery::Decision;const uint32_t start=UINT32_MAX-100;
 r.update(false,start,start,false);r.update(true,start+100,start+100,true);
 TEST_ASSERT_EQUAL((int)D::Proceed,(int)r.update(true,start+210,start+210,true));
}
void test_sensor_pause_tracks_coasting_then_completes_original_cell(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();
 const auto left=[&](float mm){return lroundf(mm*f.cfg.leftCountsPerMm);};
 const auto right=[&](float mm){return lroundf(mm*f.cfg.rightCountsPerMm);};
 f.tick(left(30),right(30),0);
 TEST_ASSERT_TRUE(f.motion.setSensorPaused(true));TEST_ASSERT_TRUE(f.braking());
 TEST_ASSERT_FALSE(f.motion.setSensorPaused(false));
 for(int i=0;i<200;++i)f.tick(left(33),right(33),0);
 TEST_ASSERT_TRUE(f.motion.isSensorPaused());TEST_ASSERT_TRUE(f.braking());
 TEST_ASSERT_EQUAL((int)MotionStatus::Running,(int)f.motion.getStatus());
 TEST_ASSERT_FLOAT_WITHIN(1,33,f.motion.getDistanceTravelledMm());
 TEST_ASSERT_TRUE(f.motion.setSensorPaused(false));
 for(int i=1;i<550;++i){float mm=33+i*.3f;f.tick(left(mm),right(mm),0);if(f.braking())break;}
 TEST_ASSERT_TRUE(f.braking());f.settle(0);
 TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
 TEST_ASSERT_FLOAT_WITHIN(6,180,f.motion.getDistanceTravelledMm());
}
void test_manual_stop_while_sensor_paused_cannot_resume(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();f.motion.setSensorPaused(true);f.tick(0,0,0);
 f.motion.stop();TEST_ASSERT_FALSE(f.motion.setSensorPaused(false));
 TEST_ASSERT_FALSE(f.motors.isEnabled());TEST_ASSERT_FALSE(f.motion.isSensorPaused());
 TEST_ASSERT_EQUAL((int)MotionFailureReason::Cancelled,(int)f.motion.getFailureReason());
}
void test_sensor_pause_does_not_hide_encoder_failure(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();f.motion.setSensorPaused(true);
 encoderValid=false;f.tick(0,0,0);
 TEST_ASSERT_FALSE(f.motors.isEnabled());
 TEST_ASSERT_EQUAL((int)MotionFailureReason::EncoderReadFailed,(int)f.motion.getFailureReason());
}

void test_pause_time_is_not_mistaken_for_motion_timeout(){
 Fixture f;f.cfg.forwardTimeoutMs=1000;TEST_ASSERT_TRUE(f.motion.begin(f.cfg,{0,0,0}));
 f.arm();f.motion.startMoveLinear(10);f.tick(0,0,0);
 TEST_ASSERT_TRUE(f.motion.setSensorPaused(true));
 for(int i=0;i<220;++i)f.tick(0,0,0);
 TEST_ASSERT_TRUE(f.motion.setSensorPaused(false));
 for(int i=1;i<100;++i){const float mm=i*.2f;f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);if(f.braking())break;}
 f.settle(0);TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
}

void test_front_approach_policy_requires_measured_slow_speed_and_position(){
 TEST_ASSERT_TRUE(FrontApproach::shouldSlow(true,12));
 TEST_ASSERT_FALSE(FrontApproach::shouldSlow(false,3));
 TEST_ASSERT_FALSE(FrontApproach::shouldSlow(true,NAN));
 TEST_ASSERT_FALSE(FrontApproach::shouldSlow(true,13));
 TEST_ASSERT_TRUE(FrontApproach::slowEnough(20,20));
 TEST_ASSERT_FALSE(FrontApproach::slowEnough(31,20));
 TEST_ASSERT_FALSE(FrontApproach::slowEnough(NAN,20));
 TEST_ASSERT_TRUE(FrontApproach::confirmed(true,3,180,180));
 TEST_ASSERT_FALSE(FrontApproach::confirmed(true,3,140,180));
 TEST_ASSERT_FALSE(FrontApproach::confirmed(true,3,201,180));
 TEST_ASSERT_FALSE(FrontApproach::confirmed(false,3,180,180));
 TEST_ASSERT_FALSE(FrontApproach::confirmed(true,4,180,180));
 TEST_ASSERT_FALSE(FrontApproach::confirmed(true,1.9f,180,180));
}
void test_front_approach_can_be_enabled_mid_segment_and_cannot_accelerate_again(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();
 for(int i=1;i<=30;++i)f.tick(i,i,0);
 const float distance=f.motion.getDistanceTravelledMm();
 f.motion.setFrontApproachMode(true);TEST_ASSERT_TRUE(f.motion.isFrontApproachMode());
 f.motion.setFrontApproachMode(false);TEST_ASSERT_TRUE(f.motion.isFrontApproachMode());
 f.tick(31,31,0);
 const float averageTarget=f.motion.getPidErrorMmS()+(f.motion.getLeftSpeedMmS()+f.motion.getRightSpeedMmS())*.5f;
 TEST_ASSERT_TRUE(averageTarget<=20.01f);
 TEST_ASSERT_TRUE(f.motion.getDistanceTravelledMm()>=distance);
}
void test_front_target_refinement_is_bounded_and_rejects_missing_or_early_readings(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();f.motion.setFrontApproachMode(true);
 TEST_ASSERT_FALSE(f.motion.refineFrontApproachTarget(4));
 for(int i=1;i<=1700;++i){const float mm=i*.1f;f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);}
 TEST_ASSERT_EQUAL((int)MotionStatus::Running,(int)f.motion.getStatus());
 TEST_ASSERT_TRUE(f.motion.refineFrontApproachTarget(4));
 TEST_ASSERT_FALSE(f.motion.refineFrontApproachTarget(NAN));
 TEST_ASSERT_FALSE(f.motion.refineFrontApproachTarget(10));
 TEST_ASSERT_FALSE(f.motion.refineFrontApproachTarget(2));
 f.motion.setSensorPaused(true);TEST_ASSERT_FALSE(f.motion.refineFrontApproachTarget(4));
}
void test_front_approach_extended_deadline_allows_slow_completion(){
 Fixture f;f.cfg.forwardTimeoutMs=500;TEST_ASSERT_TRUE(f.motion.begin(f.cfg,{0,0,0}));
 f.arm();f.motion.startMoveForwardOneCell();f.motion.setFrontApproachMode(true);
 for(int i=1;i<1800;++i){const float mm=i*.1f;f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);if(f.braking())break;}
 TEST_ASSERT_TRUE(fakeNow>500);TEST_ASSERT_TRUE(f.braking());f.settle(0);
 TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
}
void test_front_target_brake_then_corner_turn_requires_approach_mode_reset(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();f.motion.setFrontApproachMode(true);
 for(int i=1;i<=1720;++i){const float mm=i*.1f;f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);}
 TEST_ASSERT_FALSE(f.braking());
 f.motion.brakeAtFrontTarget();f.settle(0);
 TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
 f.motion.setFrontApproachMode(false);TEST_ASSERT_FALSE(f.motion.isFrontApproachMode());
 f.arm();TEST_ASSERT_TRUE(f.motion.startTurnLeft90());
}

void test_cell_observation_counts_distinct_valid_frames_after_arrival(){
 CellObservationGate gate;gate.begin(1000);
 gate.update(900,true,1000);TEST_ASSERT_FALSE(gate.ready(1000));
 gate.update(1000,true,1000);
 for(int i=0;i<10;++i)gate.update(1000,true,1000);
 TEST_ASSERT_FALSE(gate.ready(1000));
 gate.update(1100,true,1100);TEST_ASSERT_FALSE(gate.ready(1100));
 gate.update(1200,true,1200);TEST_ASSERT_TRUE(gate.ready(1200));
}
void test_cell_observation_waits_through_invalid_frame_then_recovers(){
 CellObservationGate gate;gate.begin(0);
 gate.update(0,true,0);gate.update(100,true,100);gate.update(200,false,200);
 TEST_ASSERT_FALSE(gate.ready(200));
 gate.update(300,true,300);gate.update(400,true,400);TEST_ASSERT_FALSE(gate.ready(400));
 gate.update(500,true,500);TEST_ASSERT_TRUE(gate.ready(500));
}
void test_cell_observation_timeout_and_stale_frames_never_authorize_motion(){
 CellObservationGate gate;gate.begin(0);
 gate.update(0,true,0);gate.update(100,true,100);gate.update(200,true,200);
 TEST_ASSERT_FALSE(gate.ready(200+SonarSampling::MAX_SAMPLE_AGE_MS+1));
 TEST_ASSERT_TRUE(gate.expired(1500));TEST_ASSERT_FALSE(gate.ready(1500));
 gate.begin(1600);TEST_ASSERT_FALSE(gate.ready(1600));
}
void test_cell_observation_handles_clock_wrap(){
 CellObservationGate gate;const uint32_t t=UINT32_MAX-100;gate.begin(t);
 gate.update(t,true,t);gate.update(t+100,true,t+100);gate.update(t+200,true,t+200);
 TEST_ASSERT_TRUE(gate.ready(t+200));TEST_ASSERT_TRUE(gate.expired(t+1500));
}

void test_open_corridor_cell_does_not_require_a_front_wall_at_three_cm(){
 TEST_ASSERT_TRUE(FrontApproach::arrivalConfirmed(false,true,21.1f,176.9f,180));
 TEST_ASSERT_TRUE(FrontApproach::arrivalConfirmed(false,false,NAN,176.9f,180));
 TEST_ASSERT_FALSE(FrontApproach::arrivalConfirmed(true,true,21.1f,176.9f,180));
 TEST_ASSERT_TRUE(FrontApproach::arrivalConfirmed(true,true,3.4f,176.9f,180));
 TEST_ASSERT_FALSE(FrontApproach::arrivalConfirmed(false,true,21.1f,140,180));
}

void test_median_rejects_single_short_valid_spike_and_accepts_real_wall(){
 SonarMedian3 f;f.push({true,12,696},0);TEST_ASSERT_TRUE(f.push({true,12,696},100).valid);
 TEST_ASSERT_FLOAT_WITHIN(.001f,12,f.push({true,4,232},200).distanceCm);
 TEST_ASSERT_FLOAT_WITHIN(.001f,4,f.push({true,4,232},300).distanceCm);
}
void test_median_tolerates_one_lateral_failure_but_not_repeated_loss(){
 SonarMedian3 f;f.push({true,3,174},0);f.push({true,3.1f,180},100);
 const auto held=f.push({false,NAN,0},200);
 TEST_ASSERT_TRUE(held.valid);TEST_ASSERT_FLOAT_WITHIN(.001f,3.05f,held.distanceCm);
 TEST_ASSERT_EQUAL_UINT32(100,held.lastValidAtMs);
 TEST_ASSERT_FALSE(f.push({false,NAN,0},300).valid);
 TEST_ASSERT_FALSE(f.push({false,NAN,0},400).valid);
}
void test_median_discards_below_range_one_cm_without_corrupting_twelve_cm(){
 SonarMedian3 f;f.push({true,12,696},0);f.push({true,12,696},100);
 const auto spike=f.push({false,1,58,true},200);
 TEST_ASSERT_TRUE(spike.valid);TEST_ASSERT_FLOAT_WITHIN(.001f,12,spike.distanceCm);
 TEST_ASSERT_FLOAT_WITHIN(.001f,12,f.push({true,12,696},300).distanceCm);
}
void test_median_requires_distinct_frames_and_resets_after_gap(){
 SonarMedian3 f;TEST_ASSERT_FALSE(f.push({true,12,696},0).valid);
 for(int i=0;i<10;++i)TEST_ASSERT_FALSE(f.push({true,12,696},0).valid);
 TEST_ASSERT_TRUE(f.push({true,12,696},100).valid);
 TEST_ASSERT_FALSE(f.push({true,3,174},1000).valid);
}
void test_median_freshness_and_clock_wrap_are_bounded(){
 SonarMedian3 f;const uint32_t t=UINT32_MAX-100;
 f.push({true,3,174},t);const auto value=f.push({true,3,174},t+100);
 TEST_ASSERT_TRUE(value.valid);
 SonarSnapshot s;s.hasFilters=true;s.filteredLeft=value;s.left={false,NAN,0};
 TEST_ASSERT_TRUE(s.leftEstimate(t+200).valid);TEST_ASSERT_EQUAL_UINT32(0,s.leftEstimate(t+200).pulseUs);
 TEST_ASSERT_FALSE(s.leftEstimate(t+100+SonarSampling::MAX_SAMPLE_AGE_MS+1).valid);
}
void test_proximity_brakes_first_short_echo_and_confirms_only_distinct_samples(){
 FrontProximityGuard g;using S=FrontProximityGuard::State;
 TEST_ASSERT_EQUAL((int)S::Clear,(int)g.update(false,0));
 TEST_ASSERT_EQUAL((int)S::Suspect,(int)g.update(true,100));
 for(int i=0;i<100;++i)TEST_ASSERT_EQUAL((int)S::Suspect,(int)g.update(true,100));
 TEST_ASSERT_EQUAL((int)S::Clear,(int)g.update(false,200));
 TEST_ASSERT_EQUAL((int)S::Clear,(int)g.update(false,300));
}
void test_proximity_two_of_three_close_echoes_are_not_hidden(){
 FrontProximityGuard g;using S=FrontProximityGuard::State;
 g.update(true,0);g.update(false,100);
 TEST_ASSERT_EQUAL((int)S::Confirmed,(int)g.update(true,200));
 g.reset();g.update(true,300);
 TEST_ASSERT_EQUAL((int)S::Confirmed,(int)g.update(true,400));
}
void test_isolated_short_echo_brakes_then_resumes_the_same_motion(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();f.tick(10,10,0);
 FrontProximityGuard proximity;FrontEchoRecovery recovery;SonarMedian3 median;
 median.push({true,12,696},0);median.push({true,12,696},100);
 TEST_ASSERT_TRUE(proximity.update(true,200)==FrontProximityGuard::State::Suspect);
 TEST_ASSERT_TRUE(recovery.update(false,200,200,false)==FrontEchoRecovery::Decision::Hold);
 TEST_ASSERT_TRUE(f.motion.setSensorPaused(true));TEST_ASSERT_TRUE(f.braking());
 for(int i=0;i<40;++i)f.tick(10,10,0);
 median.push({false,1,58,true},200);
 auto good=median.push({true,12,696},300);proximity.update(false,300);
 TEST_ASSERT_TRUE(recovery.update(good.valid,300,300,f.motion.sensorPauseSettled())==FrontEchoRecovery::Decision::Hold);
 good=median.push({true,12,696},400);proximity.update(false,400);
 TEST_ASSERT_TRUE(recovery.update(good.valid,400,400,f.motion.sensorPauseSettled())==FrontEchoRecovery::Decision::Proceed);
 TEST_ASSERT_TRUE(f.motion.setSensorPaused(false));f.tick(11,11,0);
 TEST_ASSERT_EQUAL((int)MotionStatus::Running,(int)f.motion.getStatus());
 TEST_ASSERT_FALSE(f.braking());TEST_ASSERT_TRUE(f.motion.getDistanceTravelledMm()>0);
}

int main(){UNITY_BEGIN();
 RUN_TEST(test_median_rejects_single_short_valid_spike_and_accepts_real_wall);
 RUN_TEST(test_median_tolerates_one_lateral_failure_but_not_repeated_loss);
 RUN_TEST(test_median_discards_below_range_one_cm_without_corrupting_twelve_cm);
 RUN_TEST(test_median_requires_distinct_frames_and_resets_after_gap);
 RUN_TEST(test_median_freshness_and_clock_wrap_are_bounded);
 RUN_TEST(test_proximity_brakes_first_short_echo_and_confirms_only_distinct_samples);
 RUN_TEST(test_proximity_two_of_three_close_echoes_are_not_hidden);
 RUN_TEST(test_isolated_short_echo_brakes_then_resumes_the_same_motion);
 RUN_TEST(test_open_corridor_cell_does_not_require_a_front_wall_at_three_cm);
 RUN_TEST(test_cell_observation_counts_distinct_valid_frames_after_arrival);
 RUN_TEST(test_cell_observation_waits_through_invalid_frame_then_recovers);
 RUN_TEST(test_cell_observation_timeout_and_stale_frames_never_authorize_motion);
 RUN_TEST(test_cell_observation_handles_clock_wrap);
 RUN_TEST(test_front_approach_policy_requires_measured_slow_speed_and_position);
 RUN_TEST(test_front_approach_can_be_enabled_mid_segment_and_cannot_accelerate_again);
 RUN_TEST(test_front_target_refinement_is_bounded_and_rejects_missing_or_early_readings);
 RUN_TEST(test_front_approach_extended_deadline_allows_slow_completion);
 RUN_TEST(test_front_target_brake_then_corner_turn_requires_approach_mode_reset);
 RUN_TEST(test_pause_time_is_not_mistaken_for_motion_timeout);
 RUN_TEST(test_echo_recovery_requires_two_new_valid_frames_and_brake_settling);
 RUN_TEST(test_echo_recovery_timeout_is_latched_and_cannot_resume_itself);
 RUN_TEST(test_echo_recovery_rejects_more_than_three_episodes_in_one_segment);
 RUN_TEST(test_echo_recovery_good_sequence_resets_on_another_invalid_frame);
 RUN_TEST(test_echo_recovery_handles_millis_wrap);
 RUN_TEST(test_sensor_pause_tracks_coasting_then_completes_original_cell);
 RUN_TEST(test_manual_stop_while_sensor_paused_cannot_resume);
 RUN_TEST(test_sensor_pause_does_not_hide_encoder_failure);
 RUN_TEST(test_start_delay_is_cancellable_and_consumed_once);
 RUN_TEST(test_start_delay_handles_millis_wrap);
 RUN_TEST(test_lateral_median_requires_three_new_frames_and_rejects_single_outlier);
 RUN_TEST(test_lateral_opening_disables_immediately_and_rebuilds_window);
 RUN_TEST(test_lateral_invalid_echo_or_nan_clears_history);
 RUN_TEST(test_lateral_stale_frame_disables_even_if_timestamp_repeated);
 RUN_TEST(test_lateral_pause_gap_and_explicit_reset_need_fresh_window);
 RUN_TEST(test_lateral_median_handles_millis_wrap);
 RUN_TEST(test_lateral_small_difference_does_not_change_wheel_command);RUN_TEST(test_calibration_cannot_hide_very_short_raw_echo);RUN_TEST(test_sonar_calibrates_before_minimum_range_check);RUN_TEST(test_sonar_no_echo_stays_invalid_with_positive_offset);
 RUN_TEST(test_sonar_preserves_too_close_echo_without_claiming_accuracy);RUN_TEST(test_sonar_rejects_invalid_calibration);
 RUN_TEST(test_sonar_provisional_offsets_fit_reference_band);RUN_TEST(test_pre_corner_distance_respects_wall_and_cell_budget);RUN_TEST(test_confirmed_geometry_is_17_free_plus_1_wall);RUN_TEST(test_wall_centering_steers_away_from_closer_left_wall);
 RUN_TEST(test_opening_does_not_enter_wall_alignment_loop);RUN_TEST(test_front_approach_has_low_speed_and_can_brake_on_sensor_target);
 RUN_TEST(test_ordinary_motion_cannot_finish_via_front_target_command);RUN_TEST(test_corner_continuation_does_not_repeat_forward_cells);RUN_TEST(test_guided_route_is_two_forward_approach_left_forward);RUN_TEST(test_guided_route_cannot_advance_after_failure);RUN_TEST(test_reverse_signed_motion_keeps_heading_and_brakes);RUN_TEST(test_nonfinite_imu_never_generates_motor_output);RUN_TEST(test_motor_mode_matches_bench_and_disabled_boot);RUN_TEST(test_unarmed_move_rejected);
 RUN_TEST(test_linear_completes_only_after_brake_and_quiet);RUN_TEST(test_heading_correction_does_not_brake_right_for_encoder_lead);
 RUN_TEST(test_turn_180_crosses_yaw_wrap);RUN_TEST(test_turn_left_crosses_yaw_wrap);RUN_TEST(test_stalled_wheels_fail_after_braking);
 RUN_TEST(test_imu_failure_disables_immediately);RUN_TEST(test_excess_heading_fails_not_success);RUN_TEST(test_braking_that_never_settles_fails);return UNITY_END();}
