#pragma once

#include <deque>
#include <optional>

#include "vision_core/types.hpp"

namespace vision_core {

enum class BallMode {
  kLineFollow = 0,
  kApproachBall = 1,
  kTiltCameraDownAndApproach = 2,
  kFineAdjustForPickup = 3,
  kPickupBall = 4,
  kVerifyPickup = 5,
  kStandUpAfterPickup = 6,
  kReturnCameraToLine = 7,
};

enum class CameraMode {
  kForward = 0,
  kDown = 1,
  kTransition = 2,
  kGoal = 3,
};

// 외부 카메라 구동기에 이번 프레임에 전달할 요청이다.
// kNone은 발행하지 않고, kDown/kForward는 해당 자세의 도달 피드백이 올 때까지 반복한다.
enum class CameraRequest {
  kNone = 0,
  kDown,
  kForward,
  kGoal,
};

struct CameraFeedback {
  CameraMode actual_mode{CameraMode::kForward};
  bool settled{true};
};

struct BallConfig {
  int stable_window{10};
  int stable_min_hits{7};
  int lost_frames{5};
  double smooth_alpha{0.45};
  double far_u_des_norm{0.50};
  // Temporary compatibility fallback for callers that do not provide the
  // continuously-computed line vx.  New callers use far_speed_scale only.
  double far_vx{0.35};
  double far_vx_min{0.10};
  double far_wz_max{0.80};
  double far_heading_gain{2.50};
  double far_slow_by_turn{0.60};
  double far_dv_max{0.12};
  double far_dw_max{0.35};
  double far_speed_scale{0.90};
  double tilt_down_v_norm{0.75};
  int tilt_down_window{10};
  int tilt_down_min_hits{7};
  // Retained for source compatibility. The transition now depends only on the
  // rolling center-v hit history, not bbox height.
  double tilt_down_h_norm{0.18};
  double camera_tilt_duration_sec{0.50};
  double camera_settle_sec{0.25};
  double camera_return_duration_sec{0.50};
  double camera_motion_timeout_sec{3.0};
  int hold_cmd_window{5};
  double hold_vx_min{0.15};
  double hold_vx_max{0.25};
  double hold_wz_max{0.25};
  double hold_default_vx{0.18};
  double tilt_walk_speed_scale{0.50};
  double tilt_walk_vx_max{0.25};
  // 실제 미세걸음/집기/확인/일어나기 모션이 연결되기 전의 임시 동작이다.
  // 미세조정은 저속 직진으로, 나머지는 정지 명령과 시간 경과로 대신한다.
  double fine_adjust_placeholder_vx{0.15};
  double fine_adjust_placeholder_duration_sec{1.0};
  double pickup_placeholder_duration_sec{3.0};
  double pickup_verification_placeholder_sec{0.0};
  double stand_up_placeholder_sec{0.0};
  double ball_ignore_duration_sec{10.0};
  double near_target_u_norm{0.50};
  double near_target_v_norm{0.70};
  double near_kx{0.50};
  double near_ky{0.45};
  double near_wz_gain{0.80};
  double near_vx_max{0.18};
  double near_vy_max{0.15};
  double near_wz_max{0.50};
  double near_x_tol{0.06};
  double near_y_tol{0.06};
  bool near_use_lateral{true};
};

struct TrackedBall {
  bool stable{false};
  bool visible{false};
  Point2 center_px;
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double area_norm{0.0};
  double confidence{0.0};
};

struct BallResult {
  bool active{false};
  bool reached_pickup_pose{false};
  CameraRequest camera_request{CameraRequest::kNone};
  BallMode mode{BallMode::kLineFollow};
  TrackedBall tracked;
  MotionCommand command;
};

class BallController {
public:
  explicit BallController(const BallConfig &config = BallConfig{});
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec);
  // Compatibility overload: uses timed camera feedback and the caller's line
  // vx.  New camera-aware adapters should use the CameraFeedback overload.
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec,
                     double line_vx);
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec,
                     double line_vx, bool line_reference_valid);
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec,
                     double line_vx, const CameraFeedback &camera_feedback);
  // line_reference_valid must be true only when line_vx belongs to normal
  // line tracking, not to RECOV/coast/search.  The reference is still updated
  // while ball detections are ignored during cooldown.
  BallResult Compute(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height, double now_sec,
                     double line_vx, bool line_reference_valid,
                     const CameraFeedback &camera_feedback);
  static const char *ModeName(BallMode mode);
  void Reset();

private:
  static double Clamp(double value, double low, double high);
  static double LimitRate(double previous, double target, double delta);
  void UpdateTracker(const std::optional<ObjectTarget> &ball_target,
                     int image_width, int image_height);
  MotionCommand ComputeFarCommand(const TrackedBall &ball) const;
  MotionCommand ComputeTiltCommand(double now_sec) const;
  MotionCommand ComputeFineAdjustPlaceholderCommand() const;
  void PushRecentCommand(const MotionCommand &command);
  void ClearTrackingState(bool clear_line_reference = true);
  void ResetToLineFollow(bool clear_ignore, bool clear_line_reference = true);
  BallConfig config_;
  BallMode mode_{BallMode::kLineFollow};
  std::deque<bool> hit_history_;
  int lost_count_{0};
  bool has_smoothed_{false};
  TrackedBall smoothed_;
  std::deque<MotionCommand> recent_commands_;
  MotionCommand last_command_;
  double state_enter_sec_{0.0};
  std::deque<bool> tilt_trigger_history_;
  // Most recent positive vx explicitly marked as normal line tracking.  It is
  // also refreshed during the ball-ignore cooldown for the next mission.
  double last_tracking_line_vx_{0.0};
  // Captured exactly once on LINE_FOLLOW -> APPROACH_FAR. Background line
  // recovery may continue running, but its later vx must not stop ball motion.
  double latched_far_line_vx_{0.0};
  double latched_tilt_vx_{0.0};
  double ignore_ball_until_sec_{0.0};
};

} // namespace vision_core
