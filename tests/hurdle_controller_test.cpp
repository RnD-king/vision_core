#include "vision_core/hurdle_controller.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>

namespace {

using vision_core::CameraFeedback;
using vision_core::CameraMode;
using vision_core::CameraRequest;
using vision_core::HurdleActionRequest;
using vision_core::HurdleConfig;
using vision_core::HurdleController;
using vision_core::HurdleMode;
using vision_core::ObjectTarget;

ObjectTarget Target(double u_norm, double bottom_norm, double h_norm = 0.20) {
  ObjectTarget target;
  target.class_id = 4;
  target.confidence = 1.0;
  target.width_px = 20.0;
  target.height_px = h_norm * 100.0;
  target.box_px = {u_norm * 100.0 - 10.0,
                   bottom_norm * 100.0 - target.height_px,
                   target.width_px, target.height_px};
  target.center_px = {u_norm * 100.0,
                      target.box_px.y + 0.5 * target.height_px};
  target.rectified_center_px = target.center_px;
  return target;
}

CameraFeedback Forward() { return {CameraMode::kForward, true}; }
CameraFeedback Down() { return {CameraMode::kDown, true}; }
CameraFeedback Moving() { return {CameraMode::kTransition, false}; }

void TestHurdlePlaceholderSequence() {
  HurdleConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_trigger_window = 1;
  cfg.tilt_trigger_min_hits = 1;
  cfg.contact_walk_placeholder_sec = 2.0;
  cfg.cross_placeholder_sec = 3.0;
  HurdleController controller(cfg);
  const auto hurdle = Target(0.60, 0.90, 0.20);

  auto result = controller.Compute(hurdle, 100, 100, 0.0, Forward());
  assert(result.mode == HurdleMode::kTiltCameraDownAndSlow);
  assert(result.camera_request == CameraRequest::kDown);
  assert(result.command.vx > 0.0);

  result = controller.Compute(hurdle, 100, 100, 0.1, Moving());
  assert(result.mode == HurdleMode::kTiltCameraDownAndSlow);
  result = controller.Compute(hurdle, 100, 100, 0.2, Down());
  assert(result.mode == HurdleMode::kContactWalk);
  assert(result.action_request == HurdleActionRequest::kContactWalk);

  result = controller.Compute(hurdle, 100, 100, 2.19, Down());
  assert(result.mode == HurdleMode::kContactWalk);
  result = controller.Compute(hurdle, 100, 100, 2.20, Down());
  assert(result.mode == HurdleMode::kCross);
  assert(result.action_request == HurdleActionRequest::kCross);
  assert(std::abs(result.command.vx) < 1e-9);

  result = controller.Compute(hurdle, 100, 100, 5.20, Down());
  assert(result.mode == HurdleMode::kReturnCameraToLine);
  assert(result.camera_request == CameraRequest::kForward);
  result = controller.Compute(hurdle, 100, 100, 5.30, Forward());
  assert(result.mode == HurdleMode::kLineFollow);
  assert(!result.active);
}

void TestHurdleUsesBallApproachNumbers() {
  HurdleConfig cfg;
  cfg.stable_window = 1;
  cfg.stable_min_hits = 1;
  cfg.tilt_trigger_window = 1;
  cfg.tilt_trigger_min_hits = 1;
  HurdleController controller(cfg);

  const auto centered = Target(0.50, 0.60, 0.20);
  auto result =
      controller.Compute(centered, 100, 100, 0.0, 0.80, true, Forward());
  assert(result.mode == HurdleMode::kApproach);
  assert(std::abs(result.command.vx - 0.60) < 1e-9);
  assert(std::abs(result.command.wz) < 1e-9);

  controller.Reset();
  const auto down_trigger = Target(0.50, 0.85, 0.20);
  result = controller.Compute(
      down_trigger, 100, 100, 0.1, 0.80, true, Forward());
  assert(result.mode == HurdleMode::kTiltCameraDownAndSlow);
  assert(std::abs(result.command.vx - 0.25) < 1e-9);
}

} // namespace

int main() {
  TestHurdlePlaceholderSequence();
  TestHurdleUsesBallApproachNumbers();
  std::cout << "hurdle controller tests passed\n";
  return 0;
}
