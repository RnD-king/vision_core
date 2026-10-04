#include "vision_core/ball_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
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
  c.recovery_timeout_sec = 5.0; c.recovery_reacquire_min_hits = 1;
  c.recovery_center_tolerance_norm = 0.05;
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
  r = controller.Compute(Ball(0.4, 0.6), 100, 100, 0.3, down, {});
  r = controller.Compute(Ball(0.4, 0.6), 100, 100, 0.4, down, {});
  // u와 v가 모두 틀려도 lateral을 먼저 보정한다.
  assert(r.action.action == MissionAction::kLeftSideStep);
  r = controller.Compute(Ball(0.5, 0.6), 100, 100, 0.5, down, {});
  assert(r.action.action == MissionAction::kStepForwardHalf);
  r = controller.Compute(Ball(0.5, 0.8), 100, 100, 0.6, down, {});
  assert(r.action.action == MissionAction::kStepBack);
  r = controller.Compute(Ball(0.5, 0.7), 100, 100, 0.7, down, {});
  assert(r.action.action == MissionAction::kPickBall);
  assert(r.pickup_attempt_count == 1);
  r = controller.Compute(Ball(0.5, 0.7), 100, 100, 0.8, down, done);
  assert(r.action.action == MissionAction::kRecatch);

  // DOWN fine recovery는 이미 내려간 카메라를 다시 요청하지 않고 TURN만 한다.
  BallController recovery(c);
  r = recovery.Compute(Ball(0.3, 0.3), 100, 100, 0, {}, {});
  r = recovery.Compute(Ball(0.3, 0.8), 100, 100, .1, {}, done);
  r = recovery.Compute(Ball(0.3, 0.6), 100, 100, .2, down, {});
  r = recovery.Compute(std::nullopt, 100, 100, .3, down, {});
  r = recovery.Compute(std::nullopt, 100, 100, .4, down, {});
  assert(r.mode == BallMode::kBallRecoveryDown);
  r = recovery.Compute(std::nullopt, 100, 100, .5, down, {});
  assert(r.action.action == MissionAction::kTurnLeft);
  assert(r.camera_request == CameraRequest::kNone);

  // pickup 성공 뒤 STEP_BACK 완료 시 즉시 camera-forward 단계로 넘어가며
  // 같은 back-away action을 다시 발행하지 않는다.
  r = controller.Compute(std::nullopt, 100, 100, .9, down, done);
  r = controller.Compute(std::nullopt, 100, 100, 1.0, down, {});
  assert(r.action.action == MissionAction::kDefaultPosition);
  r = controller.Compute(std::nullopt, 100, 100, 1.1, down, done);
  assert(r.action.action == MissionAction::kStepBack);
  r = controller.Compute(std::nullopt, 100, 100, 1.2, down, done);
  assert(r.action.action == MissionAction::kNone);
  assert(r.camera_request == CameraRequest::kForward);
  return 0;
}
