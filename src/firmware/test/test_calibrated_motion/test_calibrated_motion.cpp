#include <unity.h>
#include <cmath>
#include <cstdint>
#include "control/motion_controller.hpp"
#include "navigation/GuidedRoute.h"

uint32_t fakeNow=0;
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
void test_guided_route_is_three_forward_left_one_forward(){
 nav::GuidedRoute route;
 const nav::GuidedAction expected[]={nav::GuidedAction::ForwardCell,nav::GuidedAction::ForwardCell,
 nav::GuidedAction::ForwardCell,nav::GuidedAction::TurnLeft90,nav::GuidedAction::ForwardCell};
 for(int i=0;i<5;++i){TEST_ASSERT_EQUAL(i,route.step());TEST_ASSERT_EQUAL((int)expected[i],(int)route.action());route.finishMovement(true);}
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::Complete,(int)route.action());route.finishMovement(true);TEST_ASSERT_EQUAL(5,route.step());}
void test_guided_route_cannot_advance_after_failure(){nav::GuidedRoute route;route.finishMovement(true);route.finishMovement(false);
 route.finishMovement(true);TEST_ASSERT_EQUAL(1,route.step());TEST_ASSERT_EQUAL((int)nav::GuidedAction::Failed,(int)route.action());
 route.reset();TEST_ASSERT_EQUAL(0,route.step());TEST_ASSERT_EQUAL((int)nav::GuidedAction::ForwardCell,(int)route.action());}
void test_corner_continuation_does_not_repeat_forward_cells(){nav::GuidedRoute route;route.resetAtCorner();
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::TurnLeft90,(int)route.action());route.finishMovement(true);
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::ForwardCell,(int)route.action());route.finishMovement(true);
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::Complete,(int)route.action());}
WallFeedback walls(float front,bool fv,float left,bool lv,float right,bool rv){
 WallFeedback w;w.frontMm=front;w.frontValid=fv;w.leftMm=left;w.leftValid=lv;w.rightMm=right;w.rightValid=rv;w.sampledAtMs=fakeNow;return w;}
void test_lateral_offset_between_two_walls_steers_toward_center(){
 // L=30 e R=10: robo deslocado 10 mm para a direita, deve virar a esquerda.
 Fixture f;f.arm();TEST_ASSERT_TRUE(f.motion.startMoveForwardOneCell());
 for(int n=1;n<=120;++n){float mm=n*0.3f;f.motion.setWallFeedback(walls(0,false,30,true,10,true));
  f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);}
 TEST_ASSERT_EQUAL((int)MotionStatus::Running,(int)f.motion.getStatus());
 TEST_ASSERT_TRUE(f.motion.getWallHeadingTargetDeg()>0.8f);
 TEST_ASSERT_TRUE(f.motion.getWallHeadingTargetDeg()<=f.cfg.maxWallHeadingDeg);
 TEST_ASSERT_TRUE(fakePwm[18]>fakePwm[16]);
 TEST_ASSERT_FLOAT_WITHIN(0.01f,20,f.motion.getLateralCenterDistanceMm());}
void test_single_wall_uses_learned_center_distance(){
 Fixture f;f.motion.setLateralCenterDistanceMm(20);f.arm();f.motion.startMoveForwardOneCell();
 // So parede direita, 8 mm mais perto que o centro: alvo de rumo a esquerda.
 for(int n=1;n<=80;++n){float mm=n*0.3f;f.motion.setWallFeedback(walls(0,false,0,false,12,true));
  f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);}
 TEST_ASSERT_TRUE(f.motion.getWallHeadingTargetDeg()>0.5f);
 // So parede esquerda, mais perto que o centro: alvo vira para a direita.
 for(int n=81;n<=200;++n){float mm=n*0.3f;f.motion.setWallFeedback(walls(0,false,12,true,0,false));
  f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);}
 TEST_ASSERT_TRUE(f.motion.getWallHeadingTargetDeg()< -0.5f);}
void test_stale_or_missing_walls_keep_imu_only(){
 Fixture f;f.arm();f.motion.startMoveForwardOneCell();
 f.motion.setWallFeedback(walls(0,false,30,true,10,true));
 for(int n=1;n<=100;++n){float mm=n*0.3f;f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);}

 TEST_ASSERT_FLOAT_WITHIN(0.3f,0,f.motion.getWallHeadingTargetDeg());}
void runToBrakeWithFront(Fixture& f,float wallAtTravelMm){
 for(int n=1;n<900 && !f.braking();++n){float mm=n*0.3f;
  f.motion.setWallFeedback(walls(f.cfg.frontStopDistanceMm+wallAtTravelMm-mm,true,0,false,0,false));
  f.tick(lroundf(mm*f.cfg.leftCountsPerMm),lroundf(mm*f.cfg.rightCountsPerMm),0);}
 TEST_ASSERT_TRUE(f.braking());f.settle(0);}
void test_front_wall_corrects_cell_end(){
 // A parede real esta 15 mm antes do que o encoder supoe.
 Fixture f;f.cfg.frontStopDistanceMm=50;TEST_ASSERT_TRUE(f.motion.begin(f.cfg,{0,0,0}));f.arm();
 TEST_ASSERT_TRUE(f.motion.startMoveForwardOneCell());runToBrakeWithFront(f,165);
 TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
 TEST_ASSERT_FLOAT_WITHIN(5,165,f.motion.getDistanceTravelledMm());
 TEST_ASSERT_FLOAT_WITHIN(3,-15,f.motion.getFrontCorrectionMm());}
void test_front_wall_can_extend_short_encoder_move(){
 Fixture f;f.cfg.frontStopDistanceMm=50;TEST_ASSERT_TRUE(f.motion.begin(f.cfg,{0,0,0}));f.arm();
 f.motion.startMoveForwardOneCell();runToBrakeWithFront(f,200);
 TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
 TEST_ASSERT_FLOAT_WITHIN(5,200,f.motion.getDistanceTravelledMm());}
void test_far_front_wall_is_ignored(){
 // Parede da celula seguinte (360 mm): fora da janela de correcao.
 Fixture f;f.cfg.frontStopDistanceMm=50;TEST_ASSERT_TRUE(f.motion.begin(f.cfg,{0,0,0}));f.arm();
 f.motion.startMoveForwardOneCell();runToBrakeWithFront(f,360);
 TEST_ASSERT_EQUAL((int)MotionStatus::Succeeded,(int)f.motion.getStatus());
 TEST_ASSERT_FLOAT_WITHIN(5,180,f.motion.getDistanceTravelledMm());
 TEST_ASSERT_FLOAT_WITHIN(0.01f,0,f.motion.getFrontCorrectionMm());}
void test_front_stop_disabled_by_default(){
 Fixture f;TEST_ASSERT_EQUAL_FLOAT(0,f.cfg.frontStopDistanceMm);f.arm();f.motion.startMoveForwardOneCell();
 runToBrakeWithFront(f,160);TEST_ASSERT_FLOAT_WITHIN(5,180,f.motion.getDistanceTravelledMm());}
void test_invalid_front_stop_config_rejected(){
 Fixture f;f.cfg.frontStopDistanceMm=10;TEST_ASSERT_FALSE(f.motion.begin(f.cfg,{0,0,0}));}
int main(){UNITY_BEGIN();
 RUN_TEST(test_lateral_offset_between_two_walls_steers_toward_center);RUN_TEST(test_single_wall_uses_learned_center_distance);
 RUN_TEST(test_stale_or_missing_walls_keep_imu_only);RUN_TEST(test_front_wall_corrects_cell_end);
 RUN_TEST(test_front_wall_can_extend_short_encoder_move);RUN_TEST(test_far_front_wall_is_ignored);
 RUN_TEST(test_front_stop_disabled_by_default);RUN_TEST(test_invalid_front_stop_config_rejected);RUN_TEST(test_corner_continuation_does_not_repeat_forward_cells);RUN_TEST(test_guided_route_is_three_forward_left_one_forward);RUN_TEST(test_guided_route_cannot_advance_after_failure);RUN_TEST(test_reverse_signed_motion_keeps_heading_and_brakes);RUN_TEST(test_nonfinite_imu_never_generates_motor_output);RUN_TEST(test_motor_mode_matches_bench_and_disabled_boot);RUN_TEST(test_unarmed_move_rejected);
 RUN_TEST(test_linear_completes_only_after_brake_and_quiet);RUN_TEST(test_heading_correction_does_not_brake_right_for_encoder_lead);
 RUN_TEST(test_turn_180_crosses_yaw_wrap);RUN_TEST(test_turn_left_crosses_yaw_wrap);RUN_TEST(test_stalled_wheels_fail_after_braking);
 RUN_TEST(test_imu_failure_disables_immediately);RUN_TEST(test_excess_heading_fails_not_success);RUN_TEST(test_braking_that_never_settles_fails);return UNITY_END();}
