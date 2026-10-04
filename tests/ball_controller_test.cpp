#include "vision_core/ball_controller.hpp"
#include <cassert>

using namespace vision_core;
static ObjectTarget Ball(double u, double v) {
  ObjectTarget t; t.center_px = {u * 100.0, v * 100.0};
  t.height_px = 10.0; t.area_px = 100.0; t.confidence = 1.0; return t;
}
int main() {
  BallConfig c{}; c.stable_window = c.stable_min_hits = 1;
  c.lost_frames = 2; c.smooth_alpha = 1.0; c.far_u_des_norm = 0.5;
  c.approach_u_deadband = 0.05; c.upper_acquire_v_norm = 1.0;
  c.tilt_down_v_norm = 0.7; c.tilt_down_window = c.tilt_down_min_hits = 1;
  c.camera_motion_timeout_sec = 3; c.fine_target_u_norm = 0.5;
  c.fine_target_v_norm = 0.7; c.fine_u_deadband = c.fine_v_deadband = 0.05;
  c.pickup_max_attempts = 2; c.pickup_success_missing_frames = 1;
  BallController controller(c);
  auto r = controller.Compute(Ball(0.3, 0.3), 100, 100, 0, {}, {});
  assert(r.mode == BallMode::kApproachBall);
  assert(r.action.action == MissionAction::kStepForwardLeft);
  ActionExecutionFeedback active; active.action_active = true;
  r = controller.Compute(Ball(0.5, 0.8), 100, 100, 0.1, {}, active);
  assert(r.camera_request == CameraRequest::kNone);
  ActionExecutionFeedback done; done.action_done = true;
  r = controller.Compute(Ball(0.5, 0.8), 100, 100, 0.2, {}, done);
  assert(r.camera_request == CameraRequest::kDown);
  CameraFeedback down{CameraMode::kDown, true};
  r = controller.Compute(Ball(0.4, 0.7), 100, 100, 0.3, down, {});
  r = controller.Compute(Ball(0.4, 0.7), 100, 100, 0.4, down, {});
  assert(r.action.action == MissionAction::kLeftSideStep);
  return 0;
}
