#pragma once

#include <deque>
#include <optional>

#include "vision_core/ball_controller.hpp"
#include "vision_core/types.hpp"

namespace vision_core {

enum class GoalMode {
  kLineFollow = 0,
  kPostPickupWait = 1,
  kTiltCameraToGoal = 2,
  kSearch = 3,
  kApproach = 4,
  kFineAdjust = 5,
  kShoot = 6,
  kReturnCameraToLine = 7,
  kHeadingRecovery = 8,
};

enum class GoalActionRequest {
  kNone = 0,
  kFineAdjust = 1,
  kShoot = 2,
};

struct GoalConfig {
  int stable_window{10};
  int stable_min_hits{7};
  int lost_frames{5};
  double smooth_alpha{0.45};
  double post_pickup_wait_sec{2.0};
  double camera_motion_timeout_sec{3.0};
  double search_wz{0.30};
  double target_u_norm{0.50};
  double approach_vx{0.30};
  double approach_wz_gain{2.0};
  double approach_wz_max{0.70};
  double fine_adjust_h_norm{0.35};
  double fine_adjust_u_tolerance{0.10};
  int fine_adjust_window{10};
  int fine_adjust_min_hits{7};
  // 실제 옆걸음/슛 모션이 연결되기 전 정지 placeholder다.
  double fine_adjust_placeholder_sec{2.0};
  double shoot_placeholder_sec{3.0};
};

struct TrackedGoal {
  bool stable{false};
  bool visible{false};
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double confidence{0.0};
};

struct GoalResult {
  bool active{false};
  CameraRequest camera_request{CameraRequest::kNone};
  GoalActionRequest action_request{GoalActionRequest::kNone};
  GoalMode mode{GoalMode::kLineFollow};
  TrackedGoal tracked;
  MotionCommand command;
};

class GoalController {
public:
  explicit GoalController(const GoalConfig &config = GoalConfig{});
  // 실제 공 집기 확인이 끝난 시점에 한 번 호출한다.
  void StartAfterPickup(double now_sec);
  GoalResult Compute(const std::optional<ObjectTarget> &goal_target,
                     int image_width, int image_height, double now_sec,
                     bool line_reference_valid,
                     const CameraFeedback &camera_feedback);
  static const char *ModeName(GoalMode mode);
  void Reset();

private:
  static double Clamp(double value, double low, double high);
  void UpdateTracker(const std::optional<ObjectTarget> &target,
                     int image_width, int image_height);
  MotionCommand ComputeApproachCommand() const;
  void ClearTracking();

  GoalConfig config_;
  GoalMode mode_{GoalMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> fine_adjust_history_;
  int lost_count_{0};
  bool has_smoothed_{false};
  TrackedGoal tracked_;
  double state_enter_sec_{0.0};
};

} // namespace vision_core
