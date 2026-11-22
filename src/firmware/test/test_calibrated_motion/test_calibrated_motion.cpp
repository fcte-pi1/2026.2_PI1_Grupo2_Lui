#include <unity.h>
#include <cmath>
#include <cstdint>
#include "control/motion_controller.hpp"
#include "navigation/GuidedRoute.h"
#include "config/maze_geometry.hpp"
#include "config/sensor_calibration.hpp"
#include "hal/ultrasonic_us100.hpp"
#include "control/corridor_feedback_filter.hpp"
#include "control/guided_lateral_clearance.hpp"
#include "control/guided_wall_recovery.hpp"
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
 void tickMm(float l,float r,float y,float g=0,uint32_t ms=5){
     tick(lroundf(l*cfg.leftCountsPerMm),lroundf(r*cfg.rightCountsPerMm),y,g,ms);
 }
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

void test_guided_pivot_syncs_both_wheels_and_targets_corridor_heading() {
    Fixture f;
    yaw = 6.0f;
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startGuidedPivot(90.0f));
    for (int step = 1; step <= 6; ++step)
        f.tickMm(-step, step, 6.0f + step, 10.0f, 100);
    TEST_ASSERT_EQUAL(HIGH, fakeDigital[17]);
    TEST_ASSERT_LESS_THAN(255, fakePwm[16]);
    TEST_ASSERT_EQUAL(LOW, fakeDigital[19]);
    TEST_ASSERT_GREATER_THAN(0, fakePwm[18]);

    float finalYaw = 12.0f;
    for (float angle = 7.0f; angle <= 84.0f && !f.braking(); angle += 0.5f) {
        const float arc = angle * 3.14159265f / 180.0f * f.cfg.wheelTrackMm * 0.5f;
        finalYaw = angle + 6.0f;
        f.tickMm(-arc, arc, finalYaw, 12.5f, 40);
    }
    TEST_ASSERT_TRUE(f.braking());
    f.settle(finalYaw);
    TEST_ASSERT_TRUE(f.motion.getStatus() == MotionStatus::Succeeded);
    TEST_ASSERT_FLOAT_WITHIN(1.2f, 90.0f, yaw);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, f.motion.getDistanceTravelledMm());
}

void test_guided_pivot_preserves_drive_and_ramp_when_the_reverse_wheel_starts_first() {
    for (const bool leftTurn : {true, false}) {
        Fixture f;
        f.arm();
        TEST_ASSERT_TRUE(f.motion.startGuidedPivot(leftTurn ? 90.0f : -90.0f));
        const int reversePin = leftTurn ? 16 : 18;
        const int forwardPin = leftTurn ? 18 : 16;
        const float direction = leftTurn ? 1.0f : -1.0f;
        for (int step = 1; step <= 5; ++step) {
            const float progress = step * 0.2f;
            const float leftMm = leftTurn ? -2.19f * progress : 0.0f;
            const float rightMm = leftTurn ? 0.75f * progress : -1.50f * progress;
            f.tickMm(leftMm, rightMm, progress * direction, 4.0f * direction, 50);
        }
        TEST_ASSERT_TRUE(f.motion.getStatus() == MotionStatus::Running);
        TEST_ASSERT_EQUAL(HIGH, fakeDigital[reversePin + 1]);
        TEST_ASSERT_LESS_THAN(255, fakePwm[reversePin]);
        TEST_ASSERT_EQUAL(LOW, fakeDigital[forwardPin + 1]);
        TEST_ASSERT_GREATER_THAN(135, fakePwm[forwardPin]);
    }
}

void test_guided_pivot_still_times_out_if_angular_progress_stops() {
    Fixture f;
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startGuidedPivot(90.0f));
    for (int sample = 1; sample <= 15 && !f.braking(); ++sample)
        f.tickMm(-0.5f * sample, 0.5f * sample, 1.0f, 0.0f, 50);
    TEST_ASSERT_TRUE(f.braking());
    f.settle(1.0f);
    TEST_ASSERT_TRUE(f.motion.getFailureReason() == MotionFailureReason::MotionTimeout);
}

void test_guided_right_pivot_reports_signed_wheel_travel() {
    Fixture f;
    yaw = -5.0f;
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startGuidedPivot(-90.0f));
    TEST_ASSERT_TRUE(f.motion.getMotionType() == MotionType::TurnRight90);
    for (float turn = 0.5f; turn <= 85.0f && !f.braking(); turn += 0.5f) {
        const float arc = turn * 0.01745329252f * f.cfg.wheelTrackMm * 0.5f;
        f.tickMm(arc, -arc, -5.0f - turn, -12.5f, 40);
    }
    TEST_ASSERT_TRUE(f.braking());
    f.settle(yaw);
    TEST_ASSERT_TRUE(f.motion.getStatus() == MotionStatus::Succeeded);
    const auto travel = f.motion.getWheelTravel();
    TEST_ASSERT_GREATER_THAN(0, travel.leftTicks);
    TEST_ASSERT_LESS_THAN(0, travel.rightTicks);
    TEST_ASSERT_FLOAT_WITHIN(1.2f, -90.0f, yaw);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, travel.leftMm + travel.rightMm);
}

void test_guided_pivot_trims_overshoot_after_settling() {
    Fixture f;
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startGuidedPivot(90.0f));
    const float mmPerDegree = 3.14159265f / 180.0f * f.cfg.wheelTrackMm * 0.5f;
    for (float angle = 0.5f; angle <= 90.0f && !f.braking(); angle += 0.5f)
        f.tickMm(-angle * mmPerDegree, angle * mmPerDegree, angle, 12.5f, 40);
    TEST_ASSERT_TRUE(f.braking());
    for (int n = 0; n < 66; ++n) f.tickMm(-92.5f * mmPerDegree, 92.5f * mmPerDegree, 92.5f);
    TEST_ASSERT_TRUE(f.motion.getStatus() == MotionStatus::Running);
    TEST_ASSERT_EQUAL(LOW, fakeDigital[17]);
    TEST_ASSERT_EQUAL(HIGH, fakeDigital[19]);
    f.tickMm(-90.2f * mmPerDegree, 90.2f * mmPerDegree, 90.2f, -8.0f);
    TEST_ASSERT_TRUE(f.braking());
    f.settle(90.2f);
    TEST_ASSERT_TRUE(f.motion.getStatus() == MotionStatus::Succeeded);
}

void test_guided_wall_recovery_counts_the_reverse_in_the_pending_target() {
    Fixture f;
    GuidedWallRecovery recovery;
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startMoveLinear(100.0f));
    f.tickMm(50.0f, 50.0f, 0.0f);
    TEST_ASSERT_TRUE(f.motion.setSensorPaused(true));
    recovery.request(false, true, false, false);
    f.motion.brakeForObstacle();
    f.settle(0.0f);
    TEST_ASSERT_TRUE(f.motion.getFailureReason() == MotionFailureReason::Obstacle);
    const float retreat = recovery.retreatDistance(f.motion.getDistanceTravelledMm(), f.motion.getLinearTargetMm());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, retreat);
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startGuidedRetreat(retreat));
    recovery.startAdjustment(40.99f);
    for (int n = 1; n <= 60 && !f.braking(); ++n)
        f.tickMm(50.0f - n * 0.5f, 50.0f - n * 0.5f, 0.0f, 0.0f, 25);
    f.settle(0.0f);
    TEST_ASSERT_TRUE(f.motion.getStatus() == MotionStatus::Succeeded);
    recovery.finishAdjustment(f.motion.getDistanceTravelledMm(), true);
    TEST_ASSERT_TRUE(recovery.phase == GuidedWallRecovery::Phase::Checking);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f - 50.0f - f.motion.getDistanceTravelledMm(), recovery.remainingMm);
    TEST_ASSERT_EQUAL_FLOAT(-6.0f, recovery.steeringOffset(3.0f, 5.0f));

    const float leftStartMm = counts[32] / f.cfg.leftCountsPerMm;
    const float rightStartMm = counts[25] / f.cfg.rightCountsPerMm;
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startMoveLinear(recovery.remainingMm));
    f.motion.setGuidedLateralSteering(true, recovery.steeringOffset(3.0f, 5.0f));
    for (int n = 1; n <= 160 && !f.braking(); ++n)
        f.tickMm(leftStartMm + n * 0.5f, rightStartMm + n * 0.5f, 0.0f, 0.0f, 25);
    f.settle(0.0f);
    TEST_ASSERT_TRUE(f.motion.getStatus() == MotionStatus::Succeeded);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 100.0f, 0.5f *
        (counts[32] / f.cfg.leftCountsPerMm + counts[25] / f.cfg.rightCountsPerMm));
}

void test_guided_wall_recovery_requires_traversed_space() {
    GuidedWallRecovery recovery;
    recovery.request(true, false, false, false);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, recovery.retreatDistance(12.0f, 180.0f));
    TEST_ASSERT_TRUE(recovery.failedAdjustmentMm < 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(6.0f, recovery.steeringOffset(7.0f, 3.0f));
}

void test_guided_corner_recovery_preserves_diagonal_and_checks_front_echo() {
    GuidedWallRecovery recovery;
    recovery.request(false, false, true, false);
    const float limitMm = nav::guidedCornerRetreatLimitMm(34.0f);
    const float retreatMm = recovery.retreatDistance(34.0f, 50.0f, limitMm);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.5f, retreatMm);
    TEST_ASSERT_TRUE(recovery.phase == GuidedWallRecovery::Phase::BeforeAdjustment);
    recovery.startAdjustment(40.99f);
    TEST_ASSERT_TRUE(recovery.frontAllowsRetreat(41.60f));
    TEST_ASSERT_TRUE(recovery.frontAllowsRetreat(40.50f));
    TEST_ASSERT_FALSE(recovery.frontAllowsRetreat(38.0f));
    TEST_ASSERT_FALSE(recovery.frontAllowsRetreat(NAN));
    recovery.finishAdjustment(-retreatMm, true);
    TEST_ASSERT_TRUE(nav::guidedCornerShiftReached(34.0f - retreatMm));

    GuidedWallRecovery frontContact;
    frontContact.request(true, false, false, false);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, frontContact.retreatDistance(47.11f, 50.0f));
    frontContact.startAdjustment(NAN);
    TEST_ASSERT_TRUE(frontContact.clearingFrontBlindZone());
    frontContact.observeFront(2.92f);
    TEST_ASSERT_FALSE(frontContact.clearingFrontBlindZone());
    TEST_ASSERT_TRUE(frontContact.frontAllowsRetreat(3.12f));
}

void test_guided_recovery_can_retry_over_previously_verified_distance() {
    GuidedWallRecovery recovery;
    recovery.request(false, false, true, false);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, recovery.retreatDistance(60.0f, 100.0f));
    recovery.startAdjustment(45.0f);
    recovery.finishAdjustment(-29.0f, true);
    for (int attempt = 0; attempt < 8; ++attempt) {
        recovery.phase = GuidedWallRecovery::Phase::Resuming;
        recovery.request(false, false, true, false);
        TEST_ASSERT_EQUAL_FLOAT(28.0f, recovery.retreatDistance(2.0f, recovery.remainingMm));
        recovery.startAdjustment(45.0f);
        recovery.finishAdjustment(-2.0f, true);
    }
    TEST_ASSERT_EQUAL_FLOAT(100.0f, recovery.originalTargetMm());
}

void test_guided_recovery_preserves_direction_and_chooses_reachable_centre() {
    GuidedWallRecovery recovery;
    recovery.request(false, false, true, false);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, recovery.retreatDistance(70.0f, 180.0f));
    recovery.startAdjustment(25.0f);
    recovery.finishAdjustment(-10.0f, false);
    TEST_ASSERT_TRUE(recovery.failedAdjustmentMm < 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(120.0f, recovery.remainingMm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, recovery.centreRetreatMm(true, 160.0f));
    recovery.planAdjustment(15.0f);
    recovery.startAdjustment(26.0f);
    recovery.finishAdjustment(0.0f, false);
    TEST_ASSERT_TRUE(recovery.failedAdjustmentMm > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(55.0f, recovery.availableRetreatMm());

    GuidedWallRecovery nearerStart;
    nearerStart.request(false, true, false, false);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, nearerStart.retreatDistance(60.0f, 180.0f));
    nearerStart.startAdjustment(30.0f);
    nearerStart.finishAdjustment(-28.0f, true);
    TEST_ASSERT_EQUAL_FLOAT(27.0f, nearerStart.centreRetreatMm(true, 35.0f));
}

void test_guided_recovery_detects_motion_against_either_linear_command() {
    for (const float direction : {-1.0f, 1.0f}) {
        Fixture f;
        f.arm();
        TEST_ASSERT_TRUE(f.motion.startMoveLinear(30.0f * direction));
        f.tickMm(-7.0f * direction, -7.0f * direction, 0.0f);
        TEST_ASSERT_TRUE(f.braking());
        f.settle(0.0f);
        TEST_ASSERT_TRUE(f.motion.getFailureReason() == MotionFailureReason::WrongDirection);
    }
}

void test_guided_centring_uses_opposite_wall_and_rejects_missing_echo() {
    GuidedLateralClearance clearance;
    float offset = 0.0f;
    TEST_ASSERT_TRUE(clearance.oppositeWallOffset(false, true, 4.0f, NAN, offset));
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, offset);
    TEST_ASSERT_TRUE(clearance.oppositeWallOffset(true, false, NAN, 4.0f, offset));
    TEST_ASSERT_LESS_THAN_FLOAT(0.0f, offset);
    TEST_ASSERT_FALSE(clearance.oppositeWallOffset(true, false, 1.8f, NAN, offset));
}
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
 nav::GuidedAction::ApproachFrontWall,nav::GuidedAction::BeginLeftTurn,nav::GuidedAction::ShiftIntoCorner,
 nav::GuidedAction::FinishLeftTurn,nav::GuidedAction::ForwardCell};
 for(int i=0;i<7;++i){TEST_ASSERT_EQUAL(i,route.step());TEST_ASSERT_EQUAL((int)expected[i],(int)route.action());route.finishMovement(true);}
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::Complete,(int)route.action());route.finishMovement(true);TEST_ASSERT_EQUAL(7,route.step());}
void test_guided_route_cannot_advance_after_failure(){nav::GuidedRoute route;route.finishMovement(true);route.finishMovement(false);
 route.finishMovement(true);TEST_ASSERT_EQUAL(1,route.step());TEST_ASSERT_EQUAL((int)nav::GuidedAction::Failed,(int)route.action());
 route.reset();TEST_ASSERT_EQUAL(0,route.step());TEST_ASSERT_EQUAL((int)nav::GuidedAction::ForwardCell,(int)route.action());}
void test_corner_continuation_does_not_repeat_forward_cells(){nav::GuidedRoute route;route.resetAtCorner();
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::ApproachFrontWall,(int)route.action());route.finishMovement(true);
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::BeginLeftTurn,(int)route.action());route.finishMovement(true);
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::ShiftIntoCorner,(int)route.action());route.finishMovement(true);
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::FinishLeftTurn,(int)route.action());route.finishMovement(true);
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::ForwardCell,(int)route.action());route.finishMovement(true);
 TEST_ASSERT_EQUAL((int)nav::GuidedAction::Complete,(int)route.action());}

void test_guided_corner_shift_limits_advance_and_credits_exit_progress() {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, nav::guidedCornerShiftMm(12.0f, 3.6f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, nav::guidedCornerShiftMm(7.6f, 6.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, nav::guidedCornerShiftMm(12.0f, NAN));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, nav::guidedCornerShiftMm(44.04f, 4.49f));
    const float shiftMm = nav::guidedCornerShiftMm(44.04f, 4.91f);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, shiftMm);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 45.3f, nav::guidedCornerExitProgressMm(shiftMm));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 43.0f, nav::guidedCornerShiftMm(9.3f, 4.91f));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, nav::guidedCornerShiftMm(12.0f, 6.0f));
    TEST_ASSERT_FALSE(nav::guidedCornerShiftReached(24.9f));
    TEST_ASSERT_TRUE(nav::guidedCornerShiftReached(25.0f));
}

void test_guided_corner_brakes_at_projected_right_wall_target() {
    TEST_ASSERT_EQUAL_FLOAT(65.0f, nav::GUIDED_CORNER_ENTRY_DEG);
    TEST_ASSERT_EQUAL_FLOAT(25.0f, nav::GUIDED_CORNER_EXIT_DEG);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, nav::guidedCornerRightGapCm(2.21f));
    TEST_ASSERT_FALSE(nav::guidedCornerBrakeAtRightTarget(5.87f, 5.50f));
    TEST_ASSERT_FALSE(nav::guidedCornerBrakeAtRightTarget(2.50f, 2.30f));
    TEST_ASSERT_TRUE(nav::guidedCornerBrakeAtRightTarget(2.27f, 2.03f));
    TEST_ASSERT_TRUE(nav::guidedCornerBrakeAtRightTarget(2.20f, NAN));
    TEST_ASSERT_FALSE(nav::guidedCornerBrakeAtRightTarget(NAN, 2.20f));
}

void test_mission_corner_requires_near_front_and_open_exit() {
    TEST_ASSERT_TRUE(nav::missionCornerEligible(3.46f, 40.0f, 180.0f));
    TEST_ASSERT_FALSE(nav::missionCornerEligible(22.0f, 40.0f, 180.0f));
    TEST_ASSERT_FALSE(nav::missionCornerEligible(2.0f, 40.0f, 180.0f));
    TEST_ASSERT_FALSE(nav::missionCornerEligible(3.46f, 4.0f, 180.0f));
    TEST_ASSERT_FALSE(nav::missionCornerEligible(3.46f, NAN, 180.0f));
}

void test_guided_corner_pivots_use_a_common_ninety_degree_heading() {
    for (const float direction : {1.0f, -1.0f}) {
        Fixture f;
        float accumulatedArc = 0.0f;
        for (const float step : {nav::GUIDED_CORNER_ENTRY_DEG, nav::GUIDED_CORNER_EXIT_DEG}) {
            const float startingYaw = yaw;
            f.arm();
            TEST_ASSERT_TRUE(f.motion.startGuidedPivot(step * direction));
            float arc = 0.0f;
            for (float turn = 0.5f; turn <= step + 1.0f && !f.braking(); turn += 0.5f) {
                arc = turn * direction * 0.01745329252f * f.cfg.wheelTrackMm * 0.5f;
                f.tickMm(-accumulatedArc - arc, accumulatedArc + arc,
                         startingYaw + turn * direction, 12.5f * direction, 40);
            }
            TEST_ASSERT_TRUE(f.braking());
            f.settle(yaw);
            TEST_ASSERT_TRUE(f.motion.getStatus() == MotionStatus::Succeeded);
            accumulatedArc += arc;
        }
        TEST_ASSERT_FLOAT_WITHIN(1.2f, 90.0f * direction, yaw);
    }
}
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
void test_guided_advance_steers_before_single_wall_reaches_steering_clearance() {
    for (const bool leftWall : {true, false}) {
        GuidedLateralClearance clearance;
        SonarSnapshot snapshot;
        auto& wall = leftWall ? snapshot.left : snapshot.right;
        wall = {true, 5.1f, 280};
        snapshot.sampledAtMs = 100;
        TEST_ASSERT_FALSE(clearance.update(snapshot).slow);
        wall.distanceCm = 4.5f;
        snapshot.sampledAtMs = 200;
        const auto feedback = clearance.update(snapshot);
        TEST_ASSERT_TRUE(feedback.slow);
        TEST_ASSERT_FALSE(feedback.leftBlocked || feedback.rightBlocked);
        TEST_ASSERT_EQUAL_FLOAT(feedback.yawOffsetDeg, clearance.update(snapshot).yawOffsetDeg);

        Fixture f;
        f.arm();
        TEST_ASSERT_TRUE(f.motion.startMoveForwardOneCell());
        f.motion.setGuidedLateralSteering(feedback.slow, feedback.yawOffsetDeg);
        for (int n = 1; n <= 4; ++n) f.tick(n, n, 0, 0, 50);
        const int outerWheel = leftWall ? 16 : 18;
        const int innerWheel = leftWall ? 18 : 16;
        TEST_ASSERT_EQUAL(LOW, fakeDigital[outerWheel + 1]);
        TEST_ASSERT_EQUAL(LOW, fakeDigital[innerWheel + 1]);
        TEST_ASSERT_GREATER_THAN(0, fakePwm[innerWheel]);
        TEST_ASSERT_GREATER_THAN(fakePwm[innerWheel], fakePwm[outerWheel]);

        wall.distanceCm = 4.6f;
        snapshot.sampledAtMs = 300;
        TEST_ASSERT_FALSE(clearance.update(snapshot).slow);
    }
}

void test_guided_advance_uses_opposite_wall_when_echo_is_lost_after_fast_closing() {
    GuidedLateralClearance clearance;
    SonarSnapshot snapshot;
    snapshot.left = {true, 4.3f, 220};
    snapshot.sampledAtMs = 100;
    TEST_ASSERT_FALSE(clearance.update(snapshot).leftBlocked);
    snapshot.left = {true, 3.0f, 145};
    snapshot.sampledAtMs = 200;
    TEST_ASSERT_FALSE(clearance.update(snapshot).leftBlocked);
    snapshot.left = {false, NAN, 0};
    snapshot.right = {true, 4.58f, 228};
    snapshot.sampledAtMs = 300;
    const auto dropped = clearance.update(snapshot);
    TEST_ASSERT_TRUE(dropped.leftUncertain);
    TEST_ASSERT_FALSE(dropped.leftBlocked);
    float offset = 0.0f;
    TEST_ASSERT_TRUE(clearance.oppositeWallOffset(dropped.leftUncertain, dropped.rightUncertain,
                                                NAN, 4.58f, offset));
    TEST_ASSERT_LESS_THAN_FLOAT(0.0f, offset);

    clearance.reset();
    snapshot.left = {true, 2.50f, 116};
    snapshot.right = {true, 2.65f, 116};
    const auto narrowCorridor = clearance.update(snapshot);
    TEST_ASSERT_TRUE(narrowCorridor.slow);
    TEST_ASSERT_FALSE(narrowCorridor.leftBlocked || narrowCorridor.rightBlocked);

    snapshot.right = {false, 2.6f, 80, true};
    snapshot.sampledAtMs = 400;
    TEST_ASSERT_TRUE(clearance.update(snapshot).rightBlocked);
    snapshot.right = {true, 1.9f, 110};
    snapshot.sampledAtMs = 500;
    TEST_ASSERT_TRUE(clearance.update(snapshot).rightBlocked);
}

void test_guided_corner_reposition_respects_clearance_and_completed_diagonal() {
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 12.0f, nav::guidedCornerRepositionMm(false, 45.0f, 3.62f, 42.8f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -4.3f, nav::guidedCornerRepositionMm(true, 45.0f, NAN, 29.8f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, nav::guidedCornerRepositionMm(false, 45.0f, 2.0f, 42.8f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, nav::guidedCornerRepositionMm(false, 45.0f, 5.5f, 63.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, nav::guidedCornerRepositionMm(true, 45.0f, 3.0f, 28.0f));
}

void test_guided_pivot_retraces_obstacle_angle_and_keeps_heading_for_retry() {
    Fixture f;
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startGuidedPivot(25.0f));
    for (int step = 1; step <= 25; ++step) {
        const float progress = step / 25.0f;
        f.tickMm(-19.10f * progress, 11.38f * progress, 14.54f * progress, 10.0f, 40);
    }
    f.motion.brakeForObstacle();
    f.settle(14.54f);
    TEST_ASSERT_TRUE(f.motion.getFailureReason() == MotionFailureReason::Obstacle);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, -3.86f, f.motion.getDistanceTravelledMm());

    f.arm();
    TEST_ASSERT_TRUE(f.motion.startGuidedPivotReturn());
    for (int step = 1; step <= 30 && !f.braking(); ++step) {
        const float remaining = 1.0f - step / 30.0f;
        f.tickMm(-19.10f * remaining, 11.38f * remaining, 14.54f * remaining, -9.0f, 50);
    }
    TEST_ASSERT_TRUE(f.braking());
    f.settle(yaw);
    TEST_ASSERT_TRUE(f.motion.getStatus() == MotionStatus::Succeeded);
    TEST_ASSERT_FLOAT_WITHIN(1.2f, 0.0f, yaw);

    const float startLeft = counts[32] / f.cfg.leftCountsPerMm;
    const float startRight = counts[25] / f.cfg.rightCountsPerMm;
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startGuidedPivot(25.0f));
    const float startYaw = yaw;
    for (float angle = startYaw + 0.5f; angle <= 25.5f && !f.braking(); angle += 0.5f) {
        const float arc = (angle - startYaw) * 0.01745329252f * f.cfg.wheelTrackMm * 0.5f;
        f.tickMm(startLeft - arc, startRight + arc, angle, 10.0f, 50);
    }
    TEST_ASSERT_TRUE(f.braking());
    f.settle(yaw);
    TEST_ASSERT_TRUE(f.motion.getStatus() == MotionStatus::Succeeded);
    TEST_ASSERT_FLOAT_WITHIN(1.2f, 25.0f, yaw);
}

void test_guided_lateral_avoidance_can_steer_during_slow_front_approach() {
    Fixture f;
    f.motion.setFrontApproachMode(true);
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startMoveLinear(25));
    f.motion.setGuidedLateralSteering(true, 3.0f);
    f.tick(0, 0, 0);
    TEST_ASSERT_EQUAL(LOW, fakeDigital[17]);
    TEST_ASSERT_EQUAL(LOW, fakeDigital[19]);
    TEST_ASSERT_GREATER_THAN(0, fakePwm[16]);
    for (int n = 1; n <= 6; ++n) f.tick(n, n, 0, 0, 50);
    TEST_ASSERT_EQUAL(LOW, fakeDigital[17]);
    TEST_ASSERT_GREATER_THAN(fakePwm[16], fakePwm[18]);
    TEST_ASSERT_TRUE(f.motion.isFrontApproachMode());
}

void test_guided_lateral_avoidance_slows_even_with_symmetric_clearance() {
    int uncorrectedPwm;
    {
        Fixture f;
        f.arm();
        TEST_ASSERT_TRUE(f.motion.startMoveForwardOneCell());
        for (int n = 1; n <= 11; ++n) f.tick(n, n, 0, 0, 50);
        uncorrectedPwm = fakePwm[16];
    }
    Fixture f;
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startMoveForwardOneCell());
    f.motion.setGuidedLateralSteering(true, 0.0f);
    for (int n = 1; n <= 11; ++n) f.tick(n, n, 0, 0, 50);
    TEST_ASSERT_TRUE(fakePwm[16] < uncorrectedPwm);
}

void test_guided_slow_approach_drives_both_wheels_and_reports_real_right_stall() {
    Fixture f;
    f.motion.setFrontApproachMode(true);
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startMoveLinear(34.9f));
    f.motion.setGuidedLateralSteering(true, -3.0f);
    f.tick(0, 0, 2.0f);
    TEST_ASSERT_EQUAL(LOW, fakeDigital[19]);
    TEST_ASSERT_GREATER_THAN(0, fakePwm[18]);
    for (int n = 1; n <= 14; ++n) f.tick(n, 0, 2.0f, 0, 50);
    f.settle(2.0f);
    TEST_ASSERT_TRUE(f.motion.getFailureReason() == MotionFailureReason::RightWheelStalled);
}

void test_guided_centering_corrects_before_clearance_limit_and_through_opening() {
    GuidedLateralClearance clearance;
    SonarSnapshot snapshot;
    snapshot.left = {true, 4.2f, 215};
    snapshot.right = {true, 6.0f, 310};
    GuidedLateralClearance::Feedback feedback;
    for (snapshot.sampledAtMs = 100; snapshot.sampledAtMs <= 300; snapshot.sampledAtMs += 100)
        feedback = clearance.update(snapshot);
    TEST_ASSERT_FALSE(feedback.slow);
    TEST_ASSERT_TRUE(feedback.yawOffsetDeg < 0.0f);

    snapshot.right = {true, 19.0f, 1050};
    TEST_ASSERT_TRUE(clearance.update(snapshot).yawOffsetDeg < 0.0f);
    snapshot.sampledAtMs = 500;
    snapshot.left = {true, 5.1f, 265};
    TEST_ASSERT_EQUAL_FLOAT(0.0f, clearance.update(snapshot).yawOffsetDeg);
}

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
void test_guided_corner_approaches_after_truncated_advance_and_long_front_echo() {
    constexpr float cell = MazeSettings::CELL_SIZE_MM;
    float remaining = 3 * cell - 175 - 120;
    // Um eco curto no trecho anterior pode encurtá-lo. A 22 cm, a
    // aproximação anterior pediria um movimento único de 190 mm.
    auto plan = nav::planGuidedCorner(22.0f, cell, remaining);
    TEST_ASSERT_TRUE(plan.action == nav::GuidedCornerAction::Advance);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 140.0f, plan.distanceMm);
    remaining -= plan.distanceMm;

    plan = nav::planGuidedCorner(8.0f, cell, remaining);
    TEST_ASSERT_TRUE(plan.action == nav::GuidedCornerAction::ApproachSlowly);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, plan.distanceMm);
    remaining -= plan.distanceMm;
    TEST_ASSERT_TRUE(nav::planGuidedCorner(3.0f, cell, remaining).action == nav::GuidedCornerAction::AtTarget);
}

void test_guided_corner_limits_travel_and_requires_room_to_turn() {
    constexpr float cell = MazeSettings::CELL_SIZE_MM;
    auto plan = nav::planGuidedCorner(45.0f, cell, 220.0f);
    TEST_ASSERT_TRUE(plan.action == nav::GuidedCornerAction::Advance);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, cell, plan.distanceMm);
    plan = nav::planGuidedCorner(27.0f, cell, 40.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, plan.distanceMm);
    TEST_ASSERT_TRUE(nav::planGuidedCorner(23.0f, cell, 0.0f).action == nav::GuidedCornerAction::NoTravelLeft);
    TEST_ASSERT_TRUE(nav::planGuidedCorner(2.4f, cell, cell).action == nav::GuidedCornerAction::TooClose);
    TEST_ASSERT_FALSE(nav::guidedLeftOpening(6.0f, cell));
    TEST_ASSERT_FALSE(nav::guidedLeftOpening(NAN, cell));
    TEST_ASSERT_TRUE(nav::guidedLeftOpening(15.0f, cell));
}

void test_guided_corner_log_reaches_slow_approach_without_a_short_restart() {
    constexpr float cell = MazeSettings::CELL_SIZE_MM;
    const float remaining = 3 * cell + FrontApproach::POSITION_TOLERANCE_MM - 178.64f - 177.52f - 147.10f;
    const auto plan = nav::planGuidedCorner(8.29f, cell, remaining);
    TEST_ASSERT_TRUE(plan.action == nav::GuidedCornerAction::ApproachSlowly);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 52.9f, plan.distanceMm);
}

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
void test_front_target_refinement_includes_progress_from_the_previous_segment() {
    Fixture f;
    f.arm();
    TEST_ASSERT_TRUE(f.motion.startMoveLinear(140.0f));
    f.motion.setFrontApproachMode(true);
    for (int i = 1; i <= 1300; ++i) {
        const float mm = i * 0.1f;
        f.tick(lroundf(mm * f.cfg.leftCountsPerMm), lroundf(mm * f.cfg.rightCountsPerMm), 0);
    }
    TEST_ASSERT_FALSE(f.motion.refineFrontApproachTarget(4.0f));
    TEST_ASSERT_TRUE(f.motion.refineFrontApproachTarget(4.0f, 40.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 140.0f, f.motion.getLinearTargetMm());
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
 RUN_TEST(test_guided_corner_brakes_at_projected_right_wall_target);
 RUN_TEST(test_mission_corner_requires_near_front_and_open_exit);
 RUN_TEST(test_guided_corner_reposition_respects_clearance_and_completed_diagonal);
 RUN_TEST(test_guided_pivot_retraces_obstacle_angle_and_keeps_heading_for_retry);
 RUN_TEST(test_guided_pivot_preserves_drive_and_ramp_when_the_reverse_wheel_starts_first);
 RUN_TEST(test_guided_pivot_still_times_out_if_angular_progress_stops);
 RUN_TEST(test_guided_right_pivot_reports_signed_wheel_travel);
 RUN_TEST(test_guided_corner_recovery_preserves_diagonal_and_checks_front_echo);
 RUN_TEST(test_guided_recovery_can_retry_over_previously_verified_distance);
 RUN_TEST(test_guided_corner_shift_limits_advance_and_credits_exit_progress);
 RUN_TEST(test_guided_corner_pivots_use_a_common_ninety_degree_heading);
 RUN_TEST(test_guided_pivot_syncs_both_wheels_and_targets_corridor_heading);
 RUN_TEST(test_guided_pivot_trims_overshoot_after_settling);
 RUN_TEST(test_guided_wall_recovery_counts_the_reverse_in_the_pending_target);
 RUN_TEST(test_guided_wall_recovery_requires_traversed_space);
 RUN_TEST(test_guided_recovery_preserves_direction_and_chooses_reachable_centre);
 RUN_TEST(test_guided_recovery_detects_motion_against_either_linear_command);
 RUN_TEST(test_guided_centring_uses_opposite_wall_and_rejects_missing_echo);
 RUN_TEST(test_guided_corner_log_reaches_slow_approach_without_a_short_restart);
 RUN_TEST(test_guided_slow_approach_drives_both_wheels_and_reports_real_right_stall);
 RUN_TEST(test_guided_centering_corrects_before_clearance_limit_and_through_opening);
 RUN_TEST(test_guided_corner_approaches_after_truncated_advance_and_long_front_echo);
 RUN_TEST(test_guided_corner_limits_travel_and_requires_room_to_turn);
 RUN_TEST(test_guided_advance_steers_before_single_wall_reaches_steering_clearance);
 RUN_TEST(test_guided_advance_uses_opposite_wall_when_echo_is_lost_after_fast_closing);
 RUN_TEST(test_guided_lateral_avoidance_can_steer_during_slow_front_approach);
 RUN_TEST(test_guided_lateral_avoidance_slows_even_with_symmetric_clearance);
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
 RUN_TEST(test_front_target_refinement_includes_progress_from_the_previous_segment);
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
