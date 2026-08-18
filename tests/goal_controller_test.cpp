#include "vision_core/goal_controller.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

namespace {

using vision_core::CameraFeedback;
using vision_core::CameraMode;
using vision_core::CameraRequest;
using vision_core::GoalActionRequest;
using vision_core::GoalConfig;
using vision_core::GoalController;
using vision_core::GoalMode;
using vision_core::GoalPoseObservation;
using vision_core::Intrinsics;
using vision_core::ObjectTarget;

ObjectTarget Target(double u_norm, double h_norm) {
  ObjectTarget target;
  target.class_id = 2;
  target.confidence = 1.0;
  target.width_px = 30.0;
  target.height_px = h_norm * 100.0;
  target.box_px = {u_norm * 100.0 - 15.0, 20.0,
                   target.width_px, target.height_px};
  target.center_px = {u_norm * 100.0,
                      target.box_px.y + 0.5 * target.height_px};
  target.rectified_center_px = target.center_px;
  return target;
}

ObjectTarget BackboardTarget() {
  ObjectTarget target = Target(0.50, 0.20);
  target.class_id = 3;
  return target;
}

GoalPoseObservation Pose(double x_m, double z_m, double yaw_rad) {
  return {true, x_m, z_m, yaw_rad, 1.0};
}

CameraFeedback LineView() { return {CameraMode::kForward, true}; }
CameraFeedback GoalView() { return {CameraMode::kGoal, true}; }
CameraFeedback Moving() { return {CameraMode::kTransition, false}; }

void TestGoalApproachFineAlignAndReturnSequence() {
  GoalConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.smooth_alpha = 1.0;
  cfg.fine_adjust_window = 1;
  cfg.fine_adjust_min_hits = 1;
  cfg.fine_pulse_duration_sec = 0.10;
  cfg.fine_near_pulse_duration_sec = 0.10;
  cfg.fine_settle_duration_sec = 0.10;
  cfg.post_pickup_wait_sec = 2.0;
  cfg.shoot_placeholder_sec = 2.0;
  GoalController controller(cfg);
  controller.StartAfterPickup(0.0);

  auto result = controller.Compute(
      std::nullopt, 100, 100, 1.99, false, LineView());
  assert(result.mode == GoalMode::kPostPickupWait);
  result = controller.Compute(
      std::nullopt, 100, 100, 2.0, false, LineView());
  assert(result.mode == GoalMode::kTiltCameraToGoal);
  assert(result.camera_request == CameraRequest::kGoal);

  result = controller.Compute(
      std::nullopt, 100, 100, 2.1, false, Moving());
  assert(result.mode == GoalMode::kTiltCameraToGoal);
  assert(result.command.vx == cfg.camera_tilt_forward_vx);
  assert(result.command.vy == 0.0);
  assert(result.command.wz == 0.0);
  result = controller.Compute(
      std::nullopt, 100, 100, 2.2, false, GoalView());
  assert(result.mode == GoalMode::kSearch);

  const auto goal = Target(0.50, 0.40);
  const auto backboard = BackboardTarget();
  result = controller.Compute(goal, backboard, Pose(0.0, 1.50, 0.0),
                              100, 100, 2.22, false, GoalView());
  assert(result.mode == GoalMode::kApproach);
  result = controller.Compute(goal, backboard, Pose(0.10, 0.75, 0.20),
                              100, 100, 2.24, false, GoalView());
  assert(result.mode == GoalMode::kFineAdjust);
  assert(result.action_request == GoalActionRequest::kFineAdjust);
  assert(result.command.vx == 0.0);
  assert(result.command.vy == 0.0);
  assert(result.command.wz < 0.0);

  result = controller.Compute(goal, backboard, Pose(0.10, 0.65, 0.0),
                              100, 100, 2.26, false, GoalView());
  assert(result.mode == GoalMode::kFineAdjust);
  // 펄스 중에는 새 관측이 들어와도 처음 선택한 yaw 동작을 유지한다.
  assert(result.command.vx == 0.0);
  assert(result.command.vy == 0.0);
  assert(result.command.wz < 0.0);

  result = controller.Compute(goal, backboard, Pose(0.10, 0.65, 0.0),
                              100, 100, 2.34, false, GoalView());
  assert(result.command.vx == 0.0);
  assert(result.command.vy == 0.0);
  assert(result.command.wz == 0.0);

  result = controller.Compute(goal, backboard, Pose(0.10, 0.65, 0.0),
                              100, 100, 2.44, false, GoalView());
  assert(result.mode == GoalMode::kFineAdjust);
  assert(result.command.vx >= 0.10);
  assert(result.command.vy == 0.0);
  assert(result.command.wz == 0.0);

  result = controller.Compute(goal, backboard, Pose(0.15, 0.55, 0.0),
                              100, 100, 2.54, false, GoalView());
  assert(result.command.vx == 0.0);
  assert(result.command.vy == 0.0);
  assert(result.command.wz == 0.0);

  result = controller.Compute(goal, backboard, Pose(0.15, 0.55, 0.0),
                              100, 100, 2.64, false, GoalView());
  assert(result.command.vx == 0.0);
  assert(result.command.vy < 0.0);
  assert(result.command.wz == 0.0);

  result = controller.Compute(goal, backboard, Pose(0.0, 0.50, 0.0),
                              100, 100, 2.74, false, GoalView());
  assert(result.mode == GoalMode::kFineAdjust);
  assert(result.command.vx == 0.0);
  assert(result.command.vy == 0.0);
  assert(result.command.wz == 0.0);
  result = controller.Compute(goal, backboard, Pose(0.0, 0.50, 0.0),
                              100, 100, 2.84, false, GoalView());
  assert(result.mode == GoalMode::kShoot);
  assert(result.action_request == GoalActionRequest::kShoot);
  assert(result.command.vx == 0.0);
  assert(result.command.vy == 0.0);
  assert(result.command.wz == 0.0);

  result = controller.Compute(goal, backboard, Pose(0.0, 0.50, 0.0),
                              100, 100, 4.83, false, GoalView());
  assert(result.mode == GoalMode::kShoot);
  result = controller.Compute(goal, backboard, Pose(0.0, 0.50, 0.0),
                              100, 100, 4.84, false, GoalView());
  assert(result.mode == GoalMode::kReturnCameraToLine);
  assert(result.camera_request == CameraRequest::kForward);

  result = controller.Compute(
      std::nullopt, 100, 100, 4.86, false, LineView());
  assert(result.mode == GoalMode::kHeadingRecovery);
  assert(!result.active);
  result = controller.Compute(
      std::nullopt, 100, 100, 4.88, true, LineView());
  assert(result.mode == GoalMode::kLineFollow);
}

void TestBackboardEdgeDepthGeometry() {
  const auto pose = vision_core::EstimateGoalPoseFromEdgeDepths(
      270.0, 0.50, 370.0, 0.40, Intrinsics{500.0, 500.0, 320.0, 240.0});
  assert(pose.valid);
  assert(pose.x_m < 0.0 && pose.x_m > -0.01);
  assert(pose.z_m > 0.44 && pose.z_m < 0.46);
  assert(pose.yaw_rad > 0.0);
}

void TestApproachContinuesWithBackboardOnly() {
  GoalConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.smooth_alpha = 1.0;
  GoalController controller(cfg);
  controller.StartAfterPickup(0.0);

  controller.Compute(std::nullopt, 100, 100, 0.0, false, GoalView());
  auto result = controller.Compute(std::nullopt, 100, 100, 0.1, false,
                                   GoalView());
  assert(result.mode == GoalMode::kSearch);

  const auto goal = Target(0.50, 0.30);
  const auto backboard = BackboardTarget();
  result = controller.Compute(goal, backboard, Pose(0.0, 1.20, 0.0),
                              100, 100, 0.2, false, GoalView());
  assert(result.mode == GoalMode::kApproach);

  // 전체 골대 bbox가 사라져도 백보드가 보이면 APPROACH를 유지한다.
  result = controller.Compute(std::nullopt, backboard,
                              Pose(0.0, 1.10, 0.0), 100, 100, 0.3,
                              false, GoalView());
  assert(result.mode == GoalMode::kApproach);
  assert(result.tracked.visible);
  assert(result.command.vx > 0.0);
}

} // namespace

int main() {
  TestGoalApproachFineAlignAndReturnSequence();
  TestBackboardEdgeDepthGeometry();
  TestApproachContinuesWithBackboardOnly();
  std::cout << "goal controller tests passed\n";
  return 0;
}
