#include "vision_core/ball_controller.hpp"
#include "vision_core/c_api.h"
#include "vision_core/motion_command_selector.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <optional>

namespace {

using vision_core::BallConfig;
using vision_core::BallController;
using vision_core::BallMode;
using vision_core::CameraFeedback;
using vision_core::CameraMode;
using vision_core::CameraRequest;
using vision_core::Features;
using vision_core::MotionCommand;
using vision_core::ObjectTarget;

constexpr double kEps = 1e-9;

ObjectTarget Target(double u_norm, double v_norm, double h_norm = 0.10,
                    int width = 100, int height = 100) {
  ObjectTarget target;
  target.class_id = 1;
  target.confidence = 1.0;
  target.center_px = {u_norm * width, v_norm * height};
  target.rectified_center_px = target.center_px;
  target.width_px = h_norm * height;
  target.height_px = h_norm * height;
  target.area_px = target.width_px * target.height_px;
  return target;
}

CameraFeedback Forward() { return {CameraMode::kForward, true}; }
CameraFeedback Down() { return {CameraMode::kDown, true}; }
CameraFeedback Moving() { return {CameraMode::kTransition, false}; }

void ExpectNear(double actual, double expected) {
  if (std::abs(actual - expected) > kEps) {
    std::cerr << "expected " << expected << ", got " << actual << '\n';
  }
  assert(std::abs(actual - expected) <= kEps);
}

void TestFarSpeedAndRollingWindowTrigger() {
  BallConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.smooth_alpha = 1.0;
  cfg.tilt_down_window = 5;
  cfg.tilt_down_min_hits = 3;
  cfg.far_speed_scale = 0.9;
  cfg.tilt_walk_speed_scale = 0.5;
  cfg.tilt_walk_vx_max = 1.0;
  BallController controller(cfg);

  auto high = Target(0.90, 0.80);
  auto low = Target(0.50, 0.70, 0.95);  // huge bbox must not trigger tilt

  auto result = controller.Compute(
      high, 100, 100, 0.00, 0.8, true, Forward());
  assert(result.mode == BallMode::kApproachBall);
  ExpectNear(result.command.vx, 0.72);

  result = controller.Compute(
      low, 100, 100, 0.02, 0.8, true, Forward());
  assert(result.mode == BallMode::kApproachBall);
  ExpectNear(result.command.vx, 0.72);

  result = controller.Compute(
      high, 100, 100, 0.04, 0.8, true, Forward());
  assert(result.mode == BallMode::kApproachBall);
  result = controller.Compute(
      std::nullopt, 100, 100, 0.06, 0.8, true, Forward());
  assert(result.mode == BallMode::kApproachBall);
  // Background line control has now entered recovery and reports a positive
  // coast speed. FAR must keep the normal TRACK speed captured on entry.
  result = controller.Compute(
      high, 100, 100, 0.08, 0.12, false, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  assert(result.camera_request == CameraRequest::kDown);
  ExpectNear(result.command.vx, 0.36);
  ExpectNear(result.command.wz, 0.0);
}

void TestPositiveRecoveryCannotOverwriteTrackingReference() {
  BallConfig cfg;
  cfg.stable_window = 3;
  cfg.stable_min_hits = 3;
  cfg.smooth_alpha = 1.0;
  cfg.tilt_down_min_hits = 99;
  cfg.far_speed_scale = 0.9;
  BallController controller(cfg);
  auto target = Target(0.50, 0.50);

  auto result = controller.Compute(
      target, 100, 100, 0.00, 0.8, true, Forward());
  assert(result.mode == BallMode::kLineFollow);
  result = controller.Compute(
      target, 100, 100, 0.02, 0.12, false, Forward());
  assert(result.mode == BallMode::kLineFollow);
  result = controller.Compute(
      target, 100, 100, 0.04, 0.12, false, Forward());
  assert(result.mode == BallMode::kApproachBall);
  ExpectNear(result.command.vx, 0.72);
}

void TestRecoveryAloneIsNotAForwardReference() {
  BallConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_min_hits = 99;
  BallController controller(cfg);

  const auto result = controller.Compute(
      Target(0.50, 0.50), 100, 100, 0.0, 0.12, false, Forward());
  assert(result.mode == BallMode::kApproachBall);
  // With no normal TRACK sample, fail safe instead of treating RECOV as the
  // requested baseline walking speed.
  ExpectNear(result.command.vx, 0.0);
}

void TestDefaultStableFramesCountTowardTiltAndRawScreenWins() {
  BallConfig cfg;
  cfg.smooth_alpha = 1.0;
  BallController controller(cfg);
  auto target = Target(0.50, 0.80);

  for (int frame = 0; frame < 6; ++frame) {
    const auto result = controller.Compute(
        target, 100, 100, 0.02 * frame, 0.8, Forward());
    assert(result.mode != BallMode::kTiltCameraDownAndApproach);
  }
  auto result = controller.Compute(target, 100, 100, 0.12, 0.8, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);

  BallConfig raw_cfg;
  raw_cfg.stable_window = 1;
  raw_cfg.stable_min_hits = 1;
  raw_cfg.tilt_down_window = 1;
  raw_cfg.tilt_down_min_hits = 1;
  raw_cfg.smooth_alpha = 1.0;
  BallController raw_controller(raw_cfg);
  auto rectified_only = Target(0.50, 0.70);
  rectified_only.center_rectified = true;
  rectified_only.rectified_center_px.v = 90.0;
  result = raw_controller.Compute(
      rectified_only, 100, 100, 0.0, 0.8, Forward());
  assert(result.mode == BallMode::kApproachBall);

  auto raw_below = Target(0.50, 0.80);
  raw_below.center_rectified = true;
  raw_below.rectified_center_px.v = 20.0;
  result = raw_controller.Compute(
      raw_below, 100, 100, 0.02, 0.8, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
}

void TestBallPickupPlaceholderSequenceAndCooldown() {
  BallConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_window = 1;
  cfg.tilt_down_min_hits = 1;
  cfg.far_speed_scale = 1.0;
  cfg.tilt_walk_speed_scale = 0.5;
  cfg.tilt_walk_vx_max = 1.0;
  cfg.fine_adjust_placeholder_vx = 0.15;
  cfg.fine_adjust_placeholder_duration_sec = 1.0;
  cfg.pickup_placeholder_duration_sec = 3.0;
  cfg.pickup_verification_placeholder_sec = 0.0;
  cfg.stand_up_placeholder_sec = 0.0;
  cfg.ball_ignore_duration_sec = 30.0;
  BallController controller(cfg);
  auto target = Target(0.50, 0.80);

  auto result = controller.Compute(target, 100, 100, 0.0, 0.8, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  ExpectNear(result.command.vx, 0.4);

  result = controller.Compute(target, 100, 100, 0.1, 0.8, Moving());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  assert(result.camera_request == CameraRequest::kDown);

  result = controller.Compute(
      target, 100, 100, 0.2, 0.8, {CameraMode::kDown, false});
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  result = controller.Compute(target, 100, 100, 0.3, 0.8, Down());
  assert(result.mode == BallMode::kFineAdjustForPickup);
  assert(result.camera_request == CameraRequest::kNone);
  ExpectNear(result.command.vx, 0.15);

  result = controller.Compute(target, 100, 100, 1.29, 0.8, Down());
  assert(result.mode == BallMode::kFineAdjustForPickup);
  result = controller.Compute(target, 100, 100, 1.30, 0.8, Down());
  assert(result.mode == BallMode::kPickupBall);
  assert(result.reached_pickup_pose);
  assert(result.camera_request == CameraRequest::kNone);
  ExpectNear(result.command.vx, 0.0);

  result = controller.Compute(target, 100, 100, 4.29, 0.8, Down());
  assert(result.mode == BallMode::kPickupBall);
  result = controller.Compute(target, 100, 100, 4.30, 0.8, Down());
  assert(result.mode == BallMode::kVerifyPickup);
  assert(result.camera_request == CameraRequest::kNone);
  assert(result.active);

  result = controller.Compute(target, 100, 100, 4.32, 0.8, Down());
  assert(result.mode == BallMode::kStandUpAfterPickup);
  result = controller.Compute(target, 100, 100, 4.34, 0.8, Down());
  assert(result.mode == BallMode::kReturnCameraToLine);
  assert(result.camera_request == CameraRequest::kForward);
  result = controller.Compute(target, 100, 100, 4.40, 0.8, Moving());
  assert(result.mode == BallMode::kReturnCameraToLine);
  result = controller.Compute(target, 100, 100, 4.50, 0.8, Forward());
  assert(result.mode == BallMode::kLineFollow);
  assert(result.camera_request == CameraRequest::kNone);
  assert(!result.active);

  // The same visible ball is ignored for the full cooldown.
  result = controller.Compute(
      target, 100, 100, 34.49, 0.6, true, Forward());
  assert(result.mode == BallMode::kLineFollow);
  assert(!result.active);
  // At expiry, tracking starts fresh and can enter again with a 1-frame test
  // stability window.
  result = controller.Compute(
      target, 100, 100, 34.50, 0.12, false, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  ExpectNear(result.command.vx, 0.30);
}

void TestCameraTimeoutAndLegacyTimedFeedback() {
  BallConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_window = 1;
  cfg.tilt_down_min_hits = 1;
  cfg.camera_motion_timeout_sec = 0.5;
  BallController controller(cfg);
  auto target = Target(0.50, 0.80);

  auto result = controller.Compute(target, 100, 100, 0.0, 0.8, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  result = controller.Compute(target, 100, 100, 0.49, 0.8, Moving());
  assert(result.command.vx > 0.0);
  result = controller.Compute(target, 100, 100, 0.50, 0.8, Moving());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  assert(result.camera_request == CameraRequest::kDown);
  ExpectNear(result.command.vx, 0.0);
  result = controller.Compute(target, 100, 100, 0.60, 0.8, Down());
  assert(result.mode == BallMode::kFineAdjustForPickup);

  BallConfig legacy_cfg;
  legacy_cfg.stable_window = 1;
  legacy_cfg.stable_min_hits = 1;
  legacy_cfg.tilt_down_window = 1;
  legacy_cfg.tilt_down_min_hits = 1;
  legacy_cfg.camera_tilt_duration_sec = 0.50;
  legacy_cfg.camera_settle_sec = 0.25;
  BallController legacy(legacy_cfg);
  result = legacy.Compute(target, 100, 100, 0.0, 0.8);
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  result = legacy.Compute(target, 100, 100, 0.74, 0.8);
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  result = legacy.Compute(target, 100, 100, 0.75, 0.8);
  assert(result.mode == BallMode::kFineAdjustForPickup);
}

void TestLostFrameLimitIsExact() {
  BallConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_min_hits = 99;
  cfg.lost_frames = 2;
  BallController controller(cfg);
  auto target = Target(0.50, 0.50);

  auto result = controller.Compute(target, 100, 100, 0.0, 0.8, Forward());
  assert(result.mode == BallMode::kApproachBall);
  result = controller.Compute(std::nullopt, 100, 100, 0.1, 0.8, Forward());
  assert(result.mode == BallMode::kApproachBall);
  result = controller.Compute(
      std::nullopt, 100, 100, 0.2, 0.6, true, Forward());
  assert(result.mode == BallMode::kLineFollow);
  assert(!result.active);
  result = controller.Compute(
      target, 100, 100, 0.3, 0.12, false, Forward());
  assert(result.mode == BallMode::kApproachBall);
  ExpectNear(result.command.vx, 0.54);
}

void TestResetClearsCooldownAndState() {
  BallConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_down_window = 1;
  cfg.tilt_down_min_hits = 1;
  cfg.fine_adjust_placeholder_duration_sec = 0.0;
  cfg.pickup_placeholder_duration_sec = 0.0;
  cfg.pickup_verification_placeholder_sec = 0.0;
  cfg.stand_up_placeholder_sec = 0.0;
  cfg.ball_ignore_duration_sec = 30.0;
  BallController controller(cfg);
  auto target = Target(0.5, 0.8);
  controller.Compute(target, 100, 100, 0.0, 0.8, Forward());
  controller.Compute(target, 100, 100, 0.1, 0.8, Down());
  controller.Compute(target, 100, 100, 0.2, 0.8, Down());
  controller.Compute(target, 100, 100, 0.3, 0.8, Down());
  controller.Reset();
  const auto result = controller.Compute(
      target, 100, 100, 0.4, 0.4, true, Forward());
  assert(result.mode == BallMode::kTiltCameraDownAndApproach);
  ExpectNear(result.command.vx, 0.18);
}

void TestSelectorKeepsLineCandidateWhileBallActive() {
  vision_core::LineVelocityController line_controller;
  Features features;
  features.n_visible = 5.0;
  vision_core::BallResult ball;
  ball.active = true;
  ball.mode = BallMode::kApproachBall;
  ball.command = {0.2, 0.0, 0.1};
  const auto selected =
      vision_core::SelectMotionCommand(ball, features, line_controller);
  assert(selected.source == vision_core::CommandSource::kBall);
  ExpectNear(selected.command.vx, 0.2);

  const MotionCommand explicit_line{0.7, 0.0, -0.2};
  const auto pure = vision_core::SelectMotionCommand(ball, explicit_line);
  ExpectNear(pure.command.vx, 0.2);

  ball.active = false;
  const auto line_selected = vision_core::SelectMotionCommand(ball, explicit_line);
  ExpectNear(line_selected.command.vx, 0.7);
}

void TestMissionSelectorPriority() {
  vision_core::BallResult ball;
  vision_core::HurdleResult hurdle;
  vision_core::GoalResult goal;
  const MotionCommand line{0.7, 0.0, -0.2};

  hurdle.active = true;
  hurdle.mode = vision_core::HurdleMode::kApproach;
  hurdle.command = {0.3, 0.0, 0.1};
  auto selected = vision_core::SelectMotionCommand(ball, hurdle, goal, line);
  assert(selected.source == vision_core::CommandSource::kHurdle);
  ExpectNear(selected.command.vx, 0.3);

  ball.active = true;
  ball.mode = BallMode::kApproachBall;
  ball.command = {0.2, 0.0, -0.1};
  selected = vision_core::SelectMotionCommand(ball, hurdle, goal, line);
  assert(selected.source == vision_core::CommandSource::kBall);
  ExpectNear(selected.command.vx, 0.2);

  goal.active = true;
  goal.mode = vision_core::GoalMode::kApproach;
  goal.command = {0.1, 0.0, 0.0};
  selected = vision_core::SelectMotionCommand(ball, hurdle, goal, line);
  assert(selected.source == vision_core::CommandSource::kGoal);
  ExpectNear(selected.command.vx, 0.1);

  goal.active = false;
  goal.mode = vision_core::GoalMode::kHeadingRecovery;
  selected = vision_core::SelectMotionCommand(ball, hurdle, goal, line);
  assert(selected.source == vision_core::CommandSource::kLine);
  ExpectNear(selected.command.vx, 0.7);
}

void TestCApiV2V3LayoutAndPrecomputedSelector() {
  VisionBallControllerHandle handle = vision_ball_controller_create();
  assert(handle != nullptr);
  VisionObjectTarget target{};
  target.valid = 1;
  target.class_id = 1;
  target.confidence = 1.0;
  target.x = 45.0;
  target.y = 75.0;
  target.width = 10.0;
  target.height = 10.0;
  target.center_u = 50.0;
  target.center_v = 80.0;

  VisionBallResult result{};
  for (int frame = 0; frame < 7; ++frame) {
    result = vision_ball_controller_compute_v2(
        handle, target, 100, 100, 0.02 * frame, frame == 0 ? 0.8 : 0.0,
        static_cast<int>(CameraMode::kForward), 1);
  }
  assert(result.active == 1);
  assert(result.mode ==
         static_cast<int>(BallMode::kTiltCameraDownAndApproach));
  assert(result.request_camera_down == 1);
  ExpectNear(result.vx, 0.25);

  vision_ball_controller_reset(handle);
  for (int frame = 0; frame < 7; ++frame) {
    result = vision_ball_controller_compute_v3(
        handle, target, 100, 100, 0.02 * frame,
        frame == 0 ? 0.8 : 0.12, frame == 0 ? 1 : 0,
        static_cast<int>(CameraMode::kForward), 1);
  }
  assert(result.active == 1);
  assert(result.mode ==
         static_cast<int>(BallMode::kTiltCameraDownAndApproach));
  ExpectNear(result.vx, 0.25);

  VisionBallResult active_ball{};
  active_ball.active = 1;
  active_ball.mode = static_cast<int>(BallMode::kApproachBall);
  active_ball.vx = 0.2;
  active_ball.wz = 0.1;
  auto selected = vision_select_motion_command_v2(
      active_ball, 0.7, 0.0, -0.2);
  ExpectNear(selected.vx, 0.2);
  assert(selected.source == 1);

  active_ball.active = 0;
  selected = vision_select_motion_command_v2(
      active_ball, 0.7, 0.0, -0.2);
  ExpectNear(selected.vx, 0.7);
  ExpectNear(selected.wz, -0.2);
  assert(selected.source == 0);
  vision_ball_controller_destroy(handle);
}

}  // namespace

int main() {
  TestFarSpeedAndRollingWindowTrigger();
  TestPositiveRecoveryCannotOverwriteTrackingReference();
  TestRecoveryAloneIsNotAForwardReference();
  TestDefaultStableFramesCountTowardTiltAndRawScreenWins();
  TestBallPickupPlaceholderSequenceAndCooldown();
  TestCameraTimeoutAndLegacyTimedFeedback();
  TestLostFrameLimitIsExact();
  TestResetClearsCooldownAndState();
  TestSelectorKeepsLineCandidateWhileBallActive();
  TestMissionSelectorPriority();
  TestCApiV2V3LayoutAndPrecomputedSelector();
  std::cout << "ball controller tests passed\n";
  return 0;
}
