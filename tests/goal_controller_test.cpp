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

CameraFeedback LineView() { return {CameraMode::kForward, true}; }
CameraFeedback GoalView() { return {CameraMode::kGoal, true}; }
CameraFeedback Moving() { return {CameraMode::kTransition, false}; }

void TestGoalPlaceholderSequence() {
  GoalConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.fine_adjust_window = 1;
  cfg.fine_adjust_min_hits = 1;
  cfg.post_pickup_wait_sec = 2.0;
  cfg.fine_adjust_placeholder_sec = 2.0;
  cfg.shoot_placeholder_sec = 3.0;
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
  result = controller.Compute(
      std::nullopt, 100, 100, 2.2, false, GoalView());
  assert(result.mode == GoalMode::kSearch);

  const auto goal = Target(0.50, 0.40);
  result = controller.Compute(goal, 100, 100, 2.22, false, GoalView());
  assert(result.mode == GoalMode::kApproach);
  result = controller.Compute(goal, 100, 100, 2.24, false, GoalView());
  assert(result.mode == GoalMode::kFineAdjust);
  assert(result.action_request == GoalActionRequest::kFineAdjust);

  result = controller.Compute(goal, 100, 100, 4.24, false, GoalView());
  assert(result.mode == GoalMode::kShoot);
  assert(result.action_request == GoalActionRequest::kShoot);
  result = controller.Compute(goal, 100, 100, 7.24, false, GoalView());
  assert(result.mode == GoalMode::kReturnCameraToLine);
  assert(result.camera_request == CameraRequest::kForward);

  result = controller.Compute(
      std::nullopt, 100, 100, 7.30, false, LineView());
  assert(result.mode == GoalMode::kHeadingRecovery);
  assert(!result.active);
  result = controller.Compute(
      std::nullopt, 100, 100, 7.32, true, LineView());
  assert(result.mode == GoalMode::kLineFollow);
}

} // namespace

int main() {
  TestGoalPlaceholderSequence();
  std::cout << "goal controller tests passed\n";
  return 0;
}
