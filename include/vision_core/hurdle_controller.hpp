#pragma once

#include <deque>
#include <optional>

#include "vision_core/ball_controller.hpp"
#include "vision_core/types.hpp"

namespace vision_core {

enum class HurdleMode {
  kLineFollow = 0,
  kApproach = 1,
  kTiltCameraDownAndSlow = 2,
  kContactWalk = 3,
  kCross = 4,
  kReturnCameraToLine = 5,
};

enum class HurdleActionRequest {
  kNone = 0,
  kContactWalk = 1,
  kCross = 2,
};

struct HurdleConfig {
  int stable_window{10};
  int stable_min_hits{7};
  int lost_frames{5};
  double smooth_alpha{0.45};
  double target_u_norm{0.50};
  double approach_vx{0.30};
  double approach_wz_gain{2.0};
  double approach_wz_max{0.70};
  double tilt_trigger_bottom_norm{0.82};
  int tilt_trigger_window{10};
  int tilt_trigger_min_hits{7};
  double tilt_slow_vx{0.15};
  double camera_motion_timeout_sec{3.0};
  // 실제 잔발/허들 넘기 모션이 연결되기 전 임시 동작이다.
  double contact_walk_placeholder_vx{0.10};
  double contact_walk_placeholder_sec{2.0};
  double cross_placeholder_sec{3.0};
  double hurdle_ignore_duration_sec{5.0};
};

struct TrackedHurdle {
  bool stable{false};
  bool visible{false};
  double u_norm{0.0};
  double v_norm{0.0};
  double h_norm{0.0};
  double bottom_norm{0.0};
  double confidence{0.0};
};

struct HurdleResult {
  bool active{false};
  CameraRequest camera_request{CameraRequest::kNone};
  HurdleActionRequest action_request{HurdleActionRequest::kNone};
  HurdleMode mode{HurdleMode::kLineFollow};
  TrackedHurdle tracked;
  MotionCommand command;
};

class HurdleController {
public:
  explicit HurdleController(const HurdleConfig &config = HurdleConfig{});
  HurdleResult Compute(const std::optional<ObjectTarget> &hurdle_target,
                       int image_width, int image_height, double now_sec,
                       const CameraFeedback &camera_feedback);
  static const char *ModeName(HurdleMode mode);
  void Reset();

private:
  static double Clamp(double value, double low, double high);
  void UpdateTracker(const std::optional<ObjectTarget> &target,
                     int image_width, int image_height);
  MotionCommand ComputeApproachCommand() const;
  void ResetToLine(bool clear_ignore);

  HurdleConfig config_;
  HurdleMode mode_{HurdleMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> tilt_history_;
  int lost_count_{0};
  bool has_smoothed_{false};
  TrackedHurdle tracked_;
  double state_enter_sec_{0.0};
  double ignore_until_sec_{0.0};
};

} // namespace vision_core
