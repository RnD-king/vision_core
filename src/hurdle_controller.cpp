// 처리 순서: object_target_extractor의 hurdle 후보를 받아 라인 주행을 덮어쓸 명령을 만든다.
// 실제 잔발/넘기 정책은 외부 모션 패키지의 역할이며, 현재는 상태와 placeholder만 제공한다.

#include "vision_core/hurdle_controller.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
constexpr double kTimeEpsilon = 1e-9;
double SafeDenominator(double value) { return std::max(value, 1e-6); }
} // namespace

HurdleController::HurdleController(const HurdleConfig &config)
    : config_(config) {}

const char *HurdleController::ModeName(HurdleMode mode) {
  switch (mode) {
  case HurdleMode::kLineFollow: return "LINE_FOLLOW";
  case HurdleMode::kApproach: return "HURDLE_APPROACH";
  case HurdleMode::kTiltCameraDownAndSlow:
    return "HURDLE_CAMERA_TILT_DOWN_AND_SLOW";
  case HurdleMode::kContactWalk: return "HURDLE_CONTACT_WALK";
  case HurdleMode::kCross: return "HURDLE_CROSS";
  case HurdleMode::kReturnCameraToLine:
    return "HURDLE_CAMERA_RETURN_TO_LINE";
  }
  return "UNKNOWN";
}

double HurdleController::Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double HurdleController::LimitRate(double previous, double target,
                                   double delta) {
  return Clamp(target, previous - delta, previous + delta);
}

HurdleResult HurdleController::Compute(
    const std::optional<ObjectTarget> &hurdle_target, int image_width,
    int image_height, double now_sec, const CameraFeedback &camera_feedback) {
  return Compute(hurdle_target, image_width, image_height, now_sec,
                 config_.approach_vx, true, camera_feedback);
}

HurdleResult HurdleController::Compute(
    const std::optional<ObjectTarget> &hurdle_target, int image_width,
    int image_height, double now_sec, double line_vx,
    bool line_reference_valid, const CameraFeedback &camera_feedback) {
  if (line_reference_valid && std::isfinite(line_vx) && line_vx > 0.0) {
    last_tracking_line_vx_ = line_vx;
  }
  if (mode_ == HurdleMode::kLineFollow &&
      now_sec + kTimeEpsilon < ignore_until_sec_) {
    HurdleResult result;
    result.mode = HurdleMode::kLineFollow;
    return result;
  }
  if (mode_ == HurdleMode::kLineFollow && ignore_until_sec_ > 0.0) {
    ignore_until_sec_ = 0.0;
    hit_history_.clear();
    tilt_history_.clear();
    has_smoothed_ = false;
    tracked_ = {};
  }

  UpdateTracker(hurdle_target, image_width, image_height);
  if (mode_ == HurdleMode::kLineFollow && tracked_.stable && tracked_.visible) {
    const double reference_vx =
        last_tracking_line_vx_ > 0.0 ? last_tracking_line_vx_
                                     : config_.approach_vx;
    latched_approach_vx_ =
        reference_vx * Clamp(config_.approach_speed_scale, 0.0, 1.0);
    mode_ = HurdleMode::kApproach;
    state_enter_sec_ = now_sec;
  }
  if (mode_ == HurdleMode::kApproach &&
      lost_count_ >= std::max(1, config_.lost_frames)) {
    ResetToLine(false);
  }

  HurdleResult result;
  result.mode = mode_;
  result.tracked = tracked_;
  switch (mode_) {
  case HurdleMode::kLineFollow:
    return result;
  case HurdleMode::kApproach: {
    result.active = true;
    result.command = tracked_.visible ? ComputeApproachCommand()
                                      : MotionCommand{};
    last_command_ = result.command;
    const int hits = static_cast<int>(
        std::count(tilt_history_.begin(), tilt_history_.end(), true));
    if (hits >= std::max(1, config_.tilt_trigger_min_hits)) {
      const double scaled = std::max(0.0, result.command.vx) *
                            Clamp(config_.tilt_walk_speed_scale, 0.0, 1.0);
      latched_tilt_vx_ = Clamp(
          scaled > 0.0 ? scaled : config_.tilt_walk_default_vx,
          0.0, std::max(0.0, config_.tilt_walk_vx_max));
      mode_ = HurdleMode::kTiltCameraDownAndSlow;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kDown;
      result.command = {latched_tilt_vx_, 0.0, 0.0};
      last_command_ = result.command;
    }
    return result;
  }
  case HurdleMode::kTiltCameraDownAndSlow:
    result.active = true;
    result.camera_request = CameraRequest::kDown;
    result.command = {latched_tilt_vx_, 0.0, 0.0};
    if (camera_feedback.actual_mode == CameraMode::kDown &&
        camera_feedback.settled) {
      mode_ = HurdleMode::kContactWalk;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
      result.action_request = HurdleActionRequest::kContactWalk;
      result.command = {
          std::max(0.0, config_.contact_walk_placeholder_vx), 0.0, 0.0};
    } else if (now_sec - state_enter_sec_ + kTimeEpsilon >=
               std::max(0.0, config_.camera_motion_timeout_sec)) {
      result.command = {};
    }
    return result;
  case HurdleMode::kContactWalk:
    result.active = true;
    result.action_request = HurdleActionRequest::kContactWalk;
    result.command = {
        std::max(0.0, config_.contact_walk_placeholder_vx), 0.0, 0.0};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.contact_walk_placeholder_sec)) {
      mode_ = HurdleMode::kCross;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.action_request = HurdleActionRequest::kCross;
      result.command = {};
    }
    return result;
  case HurdleMode::kCross:
    result.active = true;
    result.action_request = HurdleActionRequest::kCross;
    result.command = {};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.cross_placeholder_sec)) {
      mode_ = HurdleMode::kReturnCameraToLine;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.action_request = HurdleActionRequest::kNone;
      result.camera_request = CameraRequest::kForward;
    }
    return result;
  case HurdleMode::kReturnCameraToLine:
    result.active = true;
    result.camera_request = CameraRequest::kForward;
    result.command = {};
    if (camera_feedback.actual_mode == CameraMode::kForward &&
        camera_feedback.settled) {
      ignore_until_sec_ =
          now_sec + std::max(0.0, config_.hurdle_ignore_duration_sec);
      ResetToLine(false);
      result = {};
      result.mode = HurdleMode::kLineFollow;
    }
    return result;
  }
  return result;
}

void HurdleController::UpdateTracker(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height) {
  const bool detected = target.has_value() && image_width > 1 && image_height > 1;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) {
    hit_history_.pop_front();
  }

  bool close = false;
  if (detected) {
    lost_count_ = 0;
    TrackedHurdle observed;
    observed.visible = true;
    const Point2 center = target->center_rectified
                              ? target->rectified_center_px
                              : target->center_px;
    observed.u_norm = Clamp(center.u / SafeDenominator(image_width), 0.0, 1.0);
    observed.v_norm = Clamp(center.v / SafeDenominator(image_height), 0.0, 1.0);
    observed.h_norm = Clamp(
        target->height_px / SafeDenominator(image_height), 0.0, 1.0);
    observed.bottom_norm = Clamp(
        (target->box_px.y + target->box_px.height) /
            SafeDenominator(image_height),
        0.0, 1.0);
    observed.confidence = target->confidence;
    const double raw_v_norm = Clamp(
        target->center_px.v / SafeDenominator(image_height), 0.0, 1.0);
    close = raw_v_norm >= config_.tilt_trigger_v_norm;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_smoothed_) {
      tracked_ = observed;
      has_smoothed_ = true;
    } else {
      tracked_.visible = true;
      tracked_.u_norm = (1.0 - alpha) * tracked_.u_norm + alpha * observed.u_norm;
      tracked_.v_norm = (1.0 - alpha) * tracked_.v_norm + alpha * observed.v_norm;
      tracked_.h_norm = (1.0 - alpha) * tracked_.h_norm + alpha * observed.h_norm;
      tracked_.bottom_norm =
          (1.0 - alpha) * tracked_.bottom_norm + alpha * observed.bottom_norm;
      tracked_.confidence = observed.confidence;
    }
  } else {
    ++lost_count_;
    tracked_.visible = false;
  }

  tilt_history_.push_back(close);
  while (static_cast<int>(tilt_history_.size()) >
         std::max(1, config_.tilt_trigger_window)) {
    tilt_history_.pop_front();
  }
  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  tracked_.stable = hits >= std::max(1, config_.stable_min_hits);
}

MotionCommand HurdleController::ComputeApproachCommand() const {
  const double u_error =
      (tracked_.u_norm - config_.target_u_norm) / 0.5;
  const double wz_raw =
      -std::abs(config_.approach_wz_max) *
      std::tanh(config_.approach_wz_gain * u_error);
  MotionCommand command;
  command.vx = std::max(0.0, latched_approach_vx_);
  command.wz = LimitRate(last_command_.wz, wz_raw, config_.approach_dw_max);
  return command;
}

void HurdleController::ResetToLine(bool clear_ignore) {
  mode_ = HurdleMode::kLineFollow;
  state_enter_sec_ = 0.0;
  hit_history_.clear();
  tilt_history_.clear();
  lost_count_ = 0;
  has_smoothed_ = false;
  tracked_ = {};
  last_command_ = {};
  latched_approach_vx_ = 0.0;
  latched_tilt_vx_ = 0.0;
  if (clear_ignore) last_tracking_line_vx_ = 0.0;
  if (clear_ignore) ignore_until_sec_ = 0.0;
}

void HurdleController::Reset() { ResetToLine(true); }

} // namespace vision_core
