// 처리 순서: 공 집기 완료 뒤 전체 골대로 접근하고, 백보드 RGB-D 자세로 미세정렬한 뒤 라인에 복귀한다.
// 실제 슛 모션은 외부 모션 패키지의 역할이며, 현재는 정렬 뒤 2초 정지로 대체한다.

#include "vision_core/goal_controller.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
constexpr double kTimeEpsilon = 1e-9;
constexpr double kGeometryEpsilon = 1e-6;
double SafeDenominator(double value) { return std::max(value, 1e-6); }
double WrapAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}
} // namespace

GoalPoseObservation EstimateGoalPoseFromEdgeDepths(
    double left_u_px, double left_depth_m, double right_u_px,
    double right_depth_m, const Intrinsics &intrinsics, double confidence) {
  GoalPoseObservation observation;
  if (!std::isfinite(left_u_px) || !std::isfinite(right_u_px) ||
      !std::isfinite(left_depth_m) || !std::isfinite(right_depth_m) ||
      !std::isfinite(intrinsics.fx) ||
      std::abs(intrinsics.fx) <= kGeometryEpsilon ||
      left_depth_m <= 0.0 || right_depth_m <= 0.0) {
    return observation;
  }

  const double left_x =
      (left_u_px - intrinsics.cx) * left_depth_m / intrinsics.fx;
  const double right_x =
      (right_u_px - intrinsics.cx) * right_depth_m / intrinsics.fx;
  const double width_x = right_x - left_x;
  if (!std::isfinite(width_x) || width_x <= kGeometryEpsilon) {
    return observation;
  }

  observation.valid = true;
  observation.x_m = 0.5 * (left_x + right_x);
  observation.z_m = 0.5 * (left_depth_m + right_depth_m);
  observation.yaw_rad =
      std::atan2(left_depth_m - right_depth_m, width_x);
  observation.confidence = confidence;
  return observation;
}

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
  return Compute(goal_target, std::nullopt, GoalPoseObservation{}, image_width,
                 image_height, now_sec, line_reference_valid, camera_feedback);
}

GoalResult GoalController::Compute(
    const std::optional<ObjectTarget> &goal_target,
    const std::optional<ObjectTarget> &backboard_target,
    const GoalPoseObservation &goal_pose, int image_width, int image_height,
    double now_sec, bool line_reference_valid,
    const CameraFeedback &camera_feedback) {
  // SEARCH 진입은 전체 골대 검출로만 결정한다. 다만 한 번 APPROACH에
  // 들어간 뒤 전체 형상이 화면 밖으로 잘려도, 내부 백보드 bbox가 남아
  // 있으면 그 중심을 대신 추적하여 접근을 계속한다.
  const bool may_track_backboard =
      mode_ == GoalMode::kApproach || mode_ == GoalMode::kFineAdjust;
  const auto &approach_target =
      (!goal_target && may_track_backboard) ? backboard_target : goal_target;
  UpdateGoalTracker(approach_target, image_width, image_height);
  UpdatePoseTracker(backboard_target, goal_pose);
  GoalResult result;
  result.mode = mode_;
  result.tracked = tracked_;
  result.pose = tracked_pose_;

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
    // 라인 위 직선 정렬이 확인된 뒤 진입하는 상태이므로, 카메라가 골대
    // 시야에 도달할 때까지 yaw를 섞지 않고 그대로 전진한다.
    result.command = {std::max(0.0, config_.camera_tilt_forward_vx), 0.0, 0.0};
    if (camera_feedback.actual_mode == CameraMode::kGoal &&
        camera_feedback.settled) {
      mode_ = GoalMode::kSearch;
      state_enter_sec_ = now_sec;
      ClearTracking();
      result.mode = mode_;
      result.camera_request = CameraRequest::kNone;
      result.tracked = {};
      result.pose = {};
    } else if (now_sec - state_enter_sec_ + kTimeEpsilon >=
               std::max(0.0, config_.camera_motion_timeout_sec)) {
      result.command = {};
    }
    return result;
  case GoalMode::kSearch:
    result.active = true;
    result.command = {0.0, 0.0, config_.search_wz};
    if (PoseReadyForFineAdjust()) {
      mode_ = GoalMode::kFineAdjust;
      state_enter_sec_ = now_sec;
      fine_adjust_history_.clear();
      ResetFineMotion();
      result.mode = mode_;
      result.action_request = GoalActionRequest::kFineAdjust;
      result.command = ComputeFineAdjustCommand(now_sec);
    } else if (tracked_.stable && tracked_.visible) {
      mode_ = GoalMode::kApproach;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.command = ComputeApproachCommand();
    }
    return result;
  case GoalMode::kApproach: {
    result.active = true;
    if (tracked_.visible) result.command = ComputeApproachCommand();
    if (PoseReadyForFineAdjust()) {
      mode_ = GoalMode::kFineAdjust;
      state_enter_sec_ = now_sec;
      fine_adjust_history_.clear();
      ResetFineMotion();
      result.mode = mode_;
      result.action_request = GoalActionRequest::kFineAdjust;
      result.command = ComputeFineAdjustCommand(now_sec);
      return result;
    }
    if (lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = GoalMode::kSearch;
      state_enter_sec_ = now_sec;
      ClearTracking();
      result.mode = mode_;
      result.command = {0.0, 0.0, config_.search_wz};
      return result;
    }
    return result;
  }
  case GoalMode::kFineAdjust: {
    result.active = true;
    result.action_request = GoalActionRequest::kFineAdjust;
    if (pose_lost_count_ >= std::max(1, config_.lost_frames)) {
      mode_ = tracked_.visible ? GoalMode::kApproach : GoalMode::kSearch;
      state_enter_sec_ = now_sec;
      fine_adjust_history_.clear();
      result.mode = mode_;
      result.action_request = GoalActionRequest::kNone;
      ResetFineMotion();
      result.command = tracked_.visible
                           ? ComputeApproachCommand()
                           : MotionCommand{0.0, 0.0, config_.search_wz};
      return result;
    }
    const int aligned_hits = static_cast<int>(std::count(
        fine_adjust_history_.begin(), fine_adjust_history_.end(), true));
    if (aligned_hits >= std::max(1, config_.fine_adjust_min_hits) &&
        FineAdjustSettled(now_sec)) {
      mode_ = GoalMode::kShoot;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.action_request = GoalActionRequest::kShoot;
      result.command = {};
      ResetFineMotion();
      return result;
    }
    if (tracked_pose_.visible) {
      result.command = ComputeFineAdjustCommand(now_sec);
    }
    return result;
  }
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
      result.pose = {};
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

void GoalController::UpdateGoalTracker(
    const std::optional<ObjectTarget> &target, int image_width,
    int image_height) {
  const bool detected = target.has_value() && image_width > 1 && image_height > 1;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) {
    hit_history_.pop_front();
  }

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

  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  tracked_.stable = hits >= std::max(1, config_.stable_min_hits);
}

void GoalController::UpdatePoseTracker(
    const std::optional<ObjectTarget> &backboard_target,
    const GoalPoseObservation &goal_pose) {
  const bool detected = backboard_target.has_value() && goal_pose.valid &&
                        std::isfinite(goal_pose.x_m) &&
                        std::isfinite(goal_pose.z_m) && goal_pose.z_m > 0.0 &&
                        std::isfinite(goal_pose.yaw_rad);
  pose_hit_history_.push_back(detected);
  while (static_cast<int>(pose_hit_history_.size()) >
         std::max(1, config_.stable_window)) {
    pose_hit_history_.pop_front();
  }

  if (detected) {
    pose_lost_count_ = 0;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_pose_smoothed_) {
      tracked_pose_.x_m = goal_pose.x_m;
      tracked_pose_.z_m = goal_pose.z_m;
      tracked_pose_.yaw_rad = goal_pose.yaw_rad;
      has_pose_smoothed_ = true;
    } else {
      tracked_pose_.x_m =
          (1.0 - alpha) * tracked_pose_.x_m + alpha * goal_pose.x_m;
      tracked_pose_.z_m =
          (1.0 - alpha) * tracked_pose_.z_m + alpha * goal_pose.z_m;
      tracked_pose_.yaw_rad = WrapAngle(
          tracked_pose_.yaw_rad +
          alpha * WrapAngle(goal_pose.yaw_rad - tracked_pose_.yaw_rad));
    }
    tracked_pose_.visible = true;
    tracked_pose_.confidence = goal_pose.confidence;
  } else {
    ++pose_lost_count_;
    tracked_pose_.visible = false;
  }

  const int pose_hits = static_cast<int>(std::count(
      pose_hit_history_.begin(), pose_hit_history_.end(), true));
  tracked_pose_.stable =
      pose_hits >= std::max(1, config_.stable_min_hits);

  fine_enter_history_.push_back(
      detected && goal_pose.z_m <= config_.fine_adjust_start_z_m);
  while (static_cast<int>(fine_enter_history_.size()) >
         std::max(1, config_.stable_window)) {
    fine_enter_history_.pop_front();
  }

  fine_adjust_history_.push_back(detected && PoseAligned());
  while (static_cast<int>(fine_adjust_history_.size()) >
         std::max(1, config_.fine_adjust_window)) {
    fine_adjust_history_.pop_front();
  }
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

MotionCommand GoalController::MakeFineAdjustPulse() const {
  MotionCommand command;
  const double yaw_error =
      WrapAngle(tracked_pose_.yaw_rad - config_.target_yaw_rad);
  // my_cv의 기존 식:
  // x' = backboard_x + hoop_radius * sin(yaw)
  // z' = backboard_z - hoop_radius * cos(yaw) - throwing_range
  const double x_error =
      tracked_pose_.x_m + config_.hoop_radius_m *
                                std::sin(tracked_pose_.yaw_rad);
  const double z_error =
      tracked_pose_.z_m - config_.hoop_radius_m *
                                std::cos(tracked_pose_.yaw_rad) -
      config_.throwing_range_m;

  // yaw가 틀어진 동안에는 회전만 한다. yaw가 허용 범위에 들어온 뒤에는
  // 회전을 섞지 않고 전후/좌우 평행이동으로 위치만 맞춘다.
  if (std::abs(yaw_error) > std::abs(config_.yaw_tolerance_rad)) {
    command.wz = Clamp(-config_.fine_wz_gain * yaw_error,
                       -std::abs(config_.fine_wz_max),
                       std::abs(config_.fine_wz_max));
    if (std::abs(command.wz) < std::abs(config_.fine_wz_min)) {
      command.wz = std::copysign(std::abs(config_.fine_wz_min), command.wz);
    }
    return command;
  }
  const double position_tolerance = std::abs(config_.position_tolerance_m);
  // yaw 정렬 뒤에는 x/z를 동시에 움직이지 않고, 현재 오차가 더 큰 축 하나만
  // 먼저 줄인다. 허용범위 안의 축도 고정하지 않아 다시 틀어지면 재조정한다.
  if (std::abs(z_error) >= std::abs(x_error)) {
    command.vx = Clamp(config_.fine_vx_gain * z_error,
                       -std::abs(config_.fine_vx_max),
                       std::abs(config_.fine_vx_max));
    if (std::abs(z_error) > position_tolerance &&
        std::abs(command.vx) < std::abs(config_.fine_translation_min)) {
      command.vx = std::copysign(
          std::abs(config_.fine_translation_min), z_error);
    }
  } else {
    // 카메라 오른쪽에 있으면 몸은 오른쪽으로 가야 하므로 body-y(+좌측)에는 음수다.
    command.vy = Clamp(-config_.fine_vy_gain * x_error,
                       -std::abs(config_.fine_vy_max),
                       std::abs(config_.fine_vy_max));
    if (std::abs(x_error) > position_tolerance &&
        std::abs(command.vy) < std::abs(config_.fine_translation_min)) {
      command.vy = std::copysign(
          std::abs(config_.fine_translation_min), -x_error);
    }
  }
  return command;
}

MotionCommand GoalController::ComputeFineAdjustCommand(double now_sec) {
  if (fine_motion_phase_ == FineMotionPhase::kPulse) {
    if (now_sec - fine_motion_phase_enter_sec_ + kTimeEpsilon <
        fine_pulse_duration_sec_) {
      // 움직이는 동안 우연히 허용범위를 통과한 관측은 완료 판정에 쓰지 않는다.
      fine_adjust_history_.clear();
      return fine_pulse_command_;
    }
    fine_motion_phase_ = FineMotionPhase::kSettle;
    fine_motion_phase_enter_sec_ = now_sec;
    fine_pulse_command_ = {};
    fine_adjust_history_.clear();
    return {};
  }

  if (fine_motion_phase_ == FineMotionPhase::kSettle) {
    if (now_sec - fine_motion_phase_enter_sec_ + kTimeEpsilon <
        std::max(0.0, config_.fine_settle_duration_sec)) {
      return {};
    }
    fine_motion_phase_ = FineMotionPhase::kReady;
  }

  if (PoseAligned()) return {};

  fine_pulse_command_ = MakeFineAdjustPulse();
  const bool translation = std::abs(fine_pulse_command_.vx) > 0.0 ||
                           std::abs(fine_pulse_command_.vy) > 0.0;
  const double x_error =
      tracked_pose_.x_m + config_.hoop_radius_m * std::sin(tracked_pose_.yaw_rad);
  const double z_error = tracked_pose_.z_m -
                         config_.hoop_radius_m * std::cos(tracked_pose_.yaw_rad) -
                         config_.throwing_range_m;
  const bool near_target = translation &&
      std::max(std::abs(x_error), std::abs(z_error)) <=
          std::abs(config_.fine_near_error_m);
  fine_pulse_duration_sec_ = std::max(
      0.0, near_target ? config_.fine_near_pulse_duration_sec
                       : config_.fine_pulse_duration_sec);
  fine_motion_phase_ = FineMotionPhase::kPulse;
  fine_motion_phase_enter_sec_ = now_sec;
  fine_adjust_history_.clear();
  return fine_pulse_command_;
}

bool GoalController::FineAdjustSettled(double now_sec) const {
  if (fine_motion_phase_ == FineMotionPhase::kReady) return true;
  if (fine_motion_phase_ != FineMotionPhase::kSettle) return false;
  return now_sec - fine_motion_phase_enter_sec_ + kTimeEpsilon >=
         std::max(0.0, config_.fine_settle_duration_sec);
}

void GoalController::ResetFineMotion() {
  fine_motion_phase_ = FineMotionPhase::kReady;
  fine_pulse_command_ = {};
  fine_motion_phase_enter_sec_ = 0.0;
  fine_pulse_duration_sec_ = 0.0;
}

bool GoalController::PoseReadyForFineAdjust() const {
  const int hits = static_cast<int>(std::count(
      fine_enter_history_.begin(), fine_enter_history_.end(), true));
  return tracked_pose_.stable && tracked_pose_.visible &&
         hits >= std::max(1, config_.stable_min_hits);
}

bool GoalController::PoseAligned() const {
  if (!tracked_pose_.visible) return false;
  const double x_error =
      tracked_pose_.x_m + config_.hoop_radius_m *
                                std::sin(tracked_pose_.yaw_rad);
  const double z_error =
      tracked_pose_.z_m - config_.hoop_radius_m *
                                std::cos(tracked_pose_.yaw_rad) -
      config_.throwing_range_m;
  return std::hypot(x_error, z_error) <=
             std::abs(config_.position_tolerance_m) &&
         std::abs(WrapAngle(tracked_pose_.yaw_rad - config_.target_yaw_rad)) <=
             std::abs(config_.yaw_tolerance_rad);
}

void GoalController::ClearTracking() {
  hit_history_.clear();
  pose_hit_history_.clear();
  fine_enter_history_.clear();
  fine_adjust_history_.clear();
  lost_count_ = 0;
  pose_lost_count_ = 0;
  has_smoothed_ = false;
  has_pose_smoothed_ = false;
  tracked_ = {};
  tracked_pose_ = {};
}

void GoalController::Reset() {
  mode_ = GoalMode::kLineFollow;
  state_enter_sec_ = 0.0;
  ResetFineMotion();
  ClearTracking();
}

} // namespace vision_core
