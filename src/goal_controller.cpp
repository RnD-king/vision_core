// 처리 순서: 공 집기 완료 알림 뒤 골대 탐색/접근/미세조정/슛/라인 복귀 상태를 계산한다.
// 실제 옆걸음과 슛은 외부 모션 패키지의 역할이며, 현재는 action_request와 정지 placeholder다.

#include "vision_core/goal_controller.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
constexpr double kTimeEpsilon = 1e-9;
double SafeDenominator(double value) { return std::max(value, 1e-6); }
} // namespace

GoalController::GoalController(const GoalConfig &config) : config_(config) {}

const char *GoalController::ModeName(GoalMode mode) {
  switch (mode) {
  case GoalMode::kLineFollow: return "LINE_FOLLOW";
  case GoalMode::kPostPickupWait: return "GOAL_POST_PICKUP_WAIT";
  case GoalMode::kTiltCameraToGoal: return "CAMERA_TILT_TO_GOAL_VIEW";
  case GoalMode::kSearch: return "GOAL_SEARCH";
  case GoalMode::kApproach: return "GOAL_APPROACH";
  case GoalMode::kFineAdjust: return "GOAL_FINE_ADJUST";
  case GoalMode::kShoot: return "GOAL_SHOOT";
  case GoalMode::kReturnCameraToLine: return "CAMERA_RETURN_TO_LINE_VIEW";
  case GoalMode::kHeadingRecovery: return "GOAL_HEADING_RECOVERY";
  }
  return "UNKNOWN";
}

double GoalController::Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

void GoalController::StartAfterPickup(double now_sec) {
  if (mode_ != GoalMode::kLineFollow) return;
  ClearTracking();
  mode_ = GoalMode::kPostPickupWait;
  state_enter_sec_ = now_sec;
}

GoalResult GoalController::Compute(
    const std::optional<ObjectTarget> &goal_target, int image_width,
    int image_height, double now_sec, bool line_reference_valid,
    const CameraFeedback &camera_feedback) {
  UpdateTracker(goal_target, image_width, image_height);
  GoalResult result;
  result.mode = mode_;
  result.tracked = tracked_;

  switch (mode_) {
  case GoalMode::kLineFollow:
    return result;
  case GoalMode::kPostPickupWait:
    result.active = true;
    result.command = {};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.post_pickup_wait_sec)) {
      mode_ = GoalMode::kTiltCameraToGoal;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kGoal;
    }
    return result;
  case GoalMode::kTiltCameraToGoal:
    result.active = true;
    result.camera_request = CameraRequest::kGoal;
    result.command = {};
    if (camera_feedback.actual_mode == CameraMode::kGoal &&
        camera_feedback.settled) {
      mode_ = GoalMode::kSearch;
      state_enter_sec_ = now_sec;
      ClearTracking();
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
      result.tracked = {};
    } else if (now_sec - state_enter_sec_ + kTimeEpsilon >=
               std::max(0.0, config_.camera_motion_timeout_sec)) {
      result.command = {};
    }
    return result;
  case GoalMode::kSearch:
    result.active = true;
    result.command = {0.0, 0.0, config_.search_wz};
    if (tracked_.stable && tracked_.visible) {
      mode_ = GoalMode::kApproach;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.command = ComputeApproachCommand();
    }
    return result;
  case GoalMode::kApproach: {
    result.active = true;
    if (tracked_.visible) result.command = ComputeApproachCommand();
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = GoalMode::kSearch;
      state_enter_sec_ = now_sec;
      ClearTracking();
      result.mode = mode_;
      result.command = {0.0, 0.0, config_.search_wz};
      return result;
    }
    const int hits = static_cast<int>(
        std::count(fine_adjust_history_.begin(),
                   fine_adjust_history_.end(), true));
    if (hits >= std::max(1, config_.fine_adjust_min_hits)) {
      mode_ = GoalMode::kFineAdjust;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.action_request = GoalActionRequest::kFineAdjust;
      result.command = {};
    }
    return result;
  }
  case GoalMode::kFineAdjust:
    result.active = true;
    result.action_request = GoalActionRequest::kFineAdjust;
    result.command = {};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.fine_adjust_placeholder_sec)) {
      mode_ = GoalMode::kShoot;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.action_request = GoalActionRequest::kShoot;
    }
    return result;
  case GoalMode::kShoot:
    result.active = true;
    result.action_request = GoalActionRequest::kShoot;
    result.command = {};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.shoot_placeholder_sec)) {
      mode_ = GoalMode::kReturnCameraToLine;
      state_enter_sec_ = now_sec;
      ClearTracking();
      result.mode = mode_;
      result.action_request = GoalActionRequest::kNone;
      result.camera_request = CameraRequest::kForward;
      result.tracked = {};
    }
    return result;
  case GoalMode::kReturnCameraToLine:
    result.active = true;
    result.camera_request = CameraRequest::kForward;
    result.command = {};
    if (camera_feedback.actual_mode == CameraMode::kForward &&
        camera_feedback.settled) {
      mode_ = GoalMode::kHeadingRecovery;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.active = false;
      result.camera_request = CameraRequest::kNone;
    }
    return result;
  case GoalMode::kHeadingRecovery:
    // 이 상태에서는 line controller의 RECOV 명령을 그대로 사용한다.
    result.active = false;
    result.command = {};
    if (line_reference_valid) {
      mode_ = GoalMode::kLineFollow;
      state_enter_sec_ = now_sec;
      ClearTracking();
      result = {};
      result.mode = GoalMode::kLineFollow;
    }
    return result;
  }
  return result;
}

void GoalController::UpdateTracker(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height) {
  const bool detected = target.has_value() && image_width > 1 && image_height > 1;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) {
    hit_history_.pop_front();
  }

  bool ready_for_fine_adjust = false;
  if (detected) {
    lost_count_ = 0;
    TrackedGoal observed;
    observed.visible = true;
    const Point2 center = target->center_rectified
                              ? target->rectified_center_px
                              : target->center_px;
    observed.u_norm = Clamp(center.u / SafeDenominator(image_width), 0.0, 1.0);
    observed.v_norm = Clamp(center.v / SafeDenominator(image_height), 0.0, 1.0);
    observed.h_norm = Clamp(
        target->height_px / SafeDenominator(image_height), 0.0, 1.0);
    observed.confidence = target->confidence;
    ready_for_fine_adjust =
        observed.h_norm >= config_.fine_adjust_h_norm &&
        std::abs(observed.u_norm - config_.target_u_norm) <=
            config_.fine_adjust_u_tolerance;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_smoothed_) {
      tracked_ = observed;
      has_smoothed_ = true;
    } else {
      tracked_.visible = true;
      tracked_.u_norm = (1.0 - alpha) * tracked_.u_norm + alpha * observed.u_norm;
      tracked_.v_norm = (1.0 - alpha) * tracked_.v_norm + alpha * observed.v_norm;
      tracked_.h_norm = (1.0 - alpha) * tracked_.h_norm + alpha * observed.h_norm;
      tracked_.confidence = observed.confidence;
    }
  } else {
    ++lost_count_;
    tracked_.visible = false;
  }

  fine_adjust_history_.push_back(ready_for_fine_adjust);
  while (static_cast<int>(fine_adjust_history_.size()) >
         std::max(1, config_.fine_adjust_window)) {
    fine_adjust_history_.pop_front();
  }
  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  tracked_.stable = hits >= std::max(1, config_.stable_min_hits);
}

MotionCommand GoalController::ComputeApproachCommand() const {
  const double u_error = tracked_.u_norm - config_.target_u_norm;
  MotionCommand command;
  command.vx = std::max(0.0, config_.approach_vx);
  command.wz = Clamp(-config_.approach_wz_gain * u_error,
                     -std::abs(config_.approach_wz_max),
                     std::abs(config_.approach_wz_max));
  return command;
}

void GoalController::ClearTracking() {
  hit_history_.clear();
  fine_adjust_history_.clear();
  lost_count_ = 0;
  has_smoothed_ = false;
  tracked_ = {};
}

void GoalController::Reset() {
  mode_ = GoalMode::kLineFollow;
  state_enter_sec_ = 0.0;
  ClearTracking();
}

} // namespace vision_core
