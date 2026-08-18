// 처리 순서: [물체 분기 2단계] object_target_extractor에서 공 후보를 고른 뒤 호출한다.
// 역할: 여러 프레임의 공 검출을 안정화하고, 공 접근/카메라 전환 상태와 속도 명령을 계산한다.
// 값 전달: 공 후보, 영상 크기, 시간, 점선 기준속도, 카메라 피드백을 받고 BallResult를 반환한다.
// 상태 보존: tracker와 현재 BallMode 등 이전 프레임 정보는 BallController 객체 안에 저장한다.
// 다음 단계: BallResult는 점선 후보 명령과 함께 motion_command_selector.cpp로 간다.

#include "vision_core/ball_controller.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
double SafeDenominator(double value) { return std::max(value, 1e-6); }
constexpr double kTimeEpsilon = 1e-9;
} // 익명 네임스페이스

BallController::BallController(const BallConfig &config) : config_(config) {}

const char *BallController::ModeName(BallMode mode) {
  switch (mode) {
  case BallMode::kLineFollow: return "LINE_FOLLOW";
  case BallMode::kApproachBall: return "BALL_APPROACH";
  case BallMode::kTiltCameraDownAndApproach:
    return "CAMERA_TILT_DOWN_AND_APPROACH";
  case BallMode::kFineAdjustForPickup: return "BALL_FINE_ADJUST";
  case BallMode::kPickupBall: return "BALL_PICKUP";
  case BallMode::kVerifyPickup: return "BALL_PICKUP_VERIFY";
  case BallMode::kStandUpAfterPickup: return "BALL_STAND_UP";
  case BallMode::kReturnCameraToLine: return "CAMERA_RETURN_TO_LINE";
  case BallMode::kBallRecoveryForward: return "BALL_RECOV_FORWARD";
  case BallMode::kBallRecoveryDown: return "BALL_RECOV_DOWN";
  }
  return "UNKNOWN";
}

double BallController::Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double BallController::LimitRate(double previous, double target, double delta) {
  return Clamp(target, previous - delta, previous + delta);
}

BallResult BallController::Compute(const std::optional<ObjectTarget> &ball_target,
                                   int image_width, int image_height,
                                   double now_sec) {
  return Compute(ball_target, image_width, image_height, now_sec,
                 config_.far_vx);
}

BallResult BallController::Compute(const std::optional<ObjectTarget> &ball_target,
                                   int image_width, int image_height,
                                   double now_sec, double line_vx) {
  return Compute(ball_target, image_width, image_height, now_sec, line_vx,
                 true);
}

BallResult BallController::Compute(const std::optional<ObjectTarget> &ball_target,
                                   int image_width, int image_height,
                                   double now_sec, double line_vx,
                                   bool line_reference_valid) {
  // 기존 호출자는 실제 카메라 상태를 전달하지 않는다. 기존 호출 방식은 시간으로
  // 카메라 상태를 추정하고, 새 연결 코드는 아래의 피드백 오버로드를 사용한다.
  CameraFeedback feedback;
  const double elapsed = std::max(0.0, now_sec - state_enter_sec_);
  switch (mode_) {
  case BallMode::kTiltCameraDownAndApproach:
    feedback.actual_mode =
        elapsed + kTimeEpsilon >=
                config_.camera_tilt_duration_sec + config_.camera_settle_sec
            ? CameraMode::kDown
            : CameraMode::kTransition;
    feedback.settled = feedback.actual_mode == CameraMode::kDown;
    break;
  case BallMode::kFineAdjustForPickup:
  case BallMode::kPickupBall:
  case BallMode::kVerifyPickup:
  case BallMode::kStandUpAfterPickup:
    feedback.actual_mode = CameraMode::kDown;
    feedback.settled = true;
    break;
  case BallMode::kReturnCameraToLine:
    feedback.actual_mode =
        elapsed + kTimeEpsilon >=
                config_.camera_return_duration_sec + config_.camera_settle_sec
            ? CameraMode::kForward
            : CameraMode::kTransition;
    feedback.settled = feedback.actual_mode == CameraMode::kForward;
    break;
  case BallMode::kLineFollow:
  case BallMode::kApproachBall:
  case BallMode::kBallRecoveryForward:
    feedback.actual_mode = CameraMode::kForward;
    feedback.settled = true;
    break;
  case BallMode::kBallRecoveryDown:
    feedback.actual_mode = CameraMode::kDown;
    feedback.settled = true;
    break;
  }
  return Compute(ball_target, image_width, image_height, now_sec, line_vx,
                 line_reference_valid, feedback);
}

BallResult BallController::Compute(const std::optional<ObjectTarget> &ball_target,
                                   int image_width, int image_height,
                                   double now_sec, double line_vx,
                                   const CameraFeedback &camera_feedback) {
  // 기존 C++/C API v2 호출자와의 호환 동작이다. 유효 여부가 따로 전달되지 않으므로
  // 양수인 점선 vx는 정상 추종에서 나온 값으로 간주한다.
  return Compute(ball_target, image_width, image_height, now_sec, line_vx,
                 true, camera_feedback);
}

BallResult BallController::Compute(const std::optional<ObjectTarget> &ball_target,
                                   int image_width, int image_height,
                                   double now_sec, double line_vx,
                                   bool line_reference_valid,
                                   const CameraFeedback &camera_feedback) {
  // 공 관측을 의도적으로 무시하는 동안에도 다음 미션에 사용할 기준속도를 갱신한다.
  // 복구/관성주행/탐색 명령은 정상 추종속도가 아니면서 양수일 수 있으므로
  // line_reference_valid에 반드시 false를 전달해야 한다.
  if (line_reference_valid && std::isfinite(line_vx) && line_vx > 0.0) {
    last_tracking_line_vx_ = line_vx;
  }

  if (mode_ == BallMode::kLineFollow &&
      now_sec + kTimeEpsilon < ignore_ball_until_sec_) {
    BallResult ignored;
    ignored.mode = BallMode::kLineFollow;
    return ignored;
  }

  if (mode_ == BallMode::kLineFollow && ignore_ball_until_sec_ > 0.0) {
    // 공 무시 시간이 끝났으므로 안정화 판정 구간을 새로 시작한다. 무시 시간 중의
    // 검출 결과가 다음 공 미션을 미리 활성화하면 안 된다.
    ignore_ball_until_sec_ = 0.0;
    ClearTrackingState(false);
  }

  // 카메라 이동 중의 bbox는 실제 물체 운동과 카메라 시야 운동을 구분할 수
  // 없으므로 성공/실패 이력 어디에도 넣지 않고 완전히 버린다.
  const bool camera_observation_valid =
      camera_feedback.actual_mode != CameraMode::kTransition &&
      camera_feedback.settled;
  if (camera_observation_valid) {
    UpdateTracker(ball_target, image_width, image_height);
  }
  const int upper_acquire_hits = static_cast<int>(
      std::count(upper_acquire_history_.begin(),
                 upper_acquire_history_.end(), true));
  if (mode_ == BallMode::kLineFollow && smoothed_.stable &&
      smoothed_.visible &&
      upper_acquire_hits >= std::max(1, config_.stable_min_hits)) {
    // 여기서 공 제어가 주 제어 상태가 된다. 안정화에는 여러 프레임이 필요하므로,
    // 직전에 복구 상태로 바뀌면서 나온 양수 속도로 덮어쓰지 않고 가장 최근의
    // 명시적으로 유효한 정상 추종속도를 고정한다. 이후 점선 복구 명령은 계속
    // 계산되지만 원거리 공 접근속도에는 사용하지 않는다.
    latched_far_line_vx_ = std::max(0.0, last_tracking_line_vx_);
    mode_ = BallMode::kApproachBall;
    state_enter_sec_ = now_sec;
  }
  if (mode_ == BallMode::kApproachBall &&
      lost_count_ >= std::max(1, config_.lost_frames)) {
    // 이미 정상 획득한 공을 잠깐 놓쳤다고 점선 복구로 넘기지 않는다.
    // 마지막 화면 좌우 위치를 기억한 BALL_RECOV가 공을 다시 찾는다.
    mode_ = camera_feedback.actual_mode == CameraMode::kDown
                ? BallMode::kBallRecoveryDown
                : BallMode::kBallRecoveryForward;
    state_enter_sec_ = now_sec;
    recovery_visible_count_ = 0;
  }
  if (mode_ == BallMode::kFineAdjustForPickup &&
      lost_count_ >= std::max(1, config_.lost_frames)) {
    mode_ = BallMode::kBallRecoveryDown;
    state_enter_sec_ = now_sec;
    recovery_visible_count_ = 0;
  }

  BallResult result;
  result.mode = mode_;
  result.tracked = smoothed_;
  switch (mode_) {
  case BallMode::kLineFollow:
    return result;
  case BallMode::kApproachBall:
    result.active = true;
    if (smoothed_.visible) {
      result.command = ComputeFarCommand(smoothed_);
      PushRecentCommand(result.command);
      // 안정화 판정에 사용한 프레임도 포함하여, 최근 판정 구간에서 보정 전 원본
      // 화면의 공 중심 v 좌표가 조건을 만족한 프레임 수를 센다.
      const int tilt_hits = static_cast<int>(
          std::count(tilt_trigger_history_.begin(),
                     tilt_trigger_history_.end(), true));
      if (tilt_hits >= std::max(1, config_.tilt_down_min_hits)) {
        const double scaled = std::max(0.0, result.command.vx) *
                              Clamp(config_.tilt_walk_speed_scale, 0.0, 1.0);
        latched_tilt_vx_ = Clamp(
            scaled > 0.0 ? scaled : config_.hold_default_vx,
            0.0, std::max(0.0, config_.tilt_walk_vx_max));
        mode_ = BallMode::kTiltCameraDownAndApproach;
        state_enter_sec_ = now_sec;
        result.mode = mode_;
        result.camera_request = CameraRequest::kDown;
        result.command = ComputeTiltCommand(now_sec);
      }
    } else {
      result.command = last_command_;
      result.command.vx *= 0.5;
      result.command.wz *= 0.5;
    }
    last_command_ = result.command;
    return result;
  case BallMode::kBallRecoveryForward:
  case BallMode::kBallRecoveryDown: {
    const bool recovery_down = mode_ == BallMode::kBallRecoveryDown;
    result.active = true;
    if (recovery_down) {
      result.camera_request = CameraRequest::kDown;
    }
    result.command = ComputeRecoveryCommand();
    last_command_ = result.command;
    if (smoothed_.visible) {
      ++recovery_visible_count_;
      if (recovery_visible_count_ >=
          std::max(1, config_.recovery_reacquire_min_hits)) {
        mode_ = recovery_down ? BallMode::kFineAdjustForPickup
                              : BallMode::kApproachBall;
        state_enter_sec_ = now_sec;
        result.mode = mode_;
        result.camera_request = CameraRequest::kNone;
        result.command = recovery_down
                             ? ComputeFineAdjustPlaceholderCommand()
                             : ComputeFarCommand(smoothed_);
        if (!recovery_down) {
          PushRecentCommand(result.command);
        }
        last_command_ = result.command;
      }
    } else {
      recovery_visible_count_ = 0;
    }
    if ((mode_ == BallMode::kBallRecoveryForward ||
         mode_ == BallMode::kBallRecoveryDown) &&
        now_sec - state_enter_sec_ + kTimeEpsilon >=
            std::max(0.0, config_.recovery_timeout_sec)) {
      if (recovery_down) {
        mode_ = BallMode::kReturnCameraToLine;
        state_enter_sec_ = now_sec;
        ClearTrackingState(false);
        result = {};
        result.active = true;
        result.mode = mode_;
        result.camera_request = CameraRequest::kForward;
      } else {
        ResetToLineFollow(false, false);
        result = {};
        result.mode = BallMode::kLineFollow;
      }
    }
    return result;
  }
  case BallMode::kTiltCameraDownAndApproach:
    result.active = true;
    result.camera_request = CameraRequest::kDown;
    result.command = ComputeTiltCommand(now_sec);
    last_command_ = result.command;
    if (camera_feedback.actual_mode == CameraMode::kDown &&
        camera_feedback.settled) {
      mode_ = smoothed_.visible ? BallMode::kFineAdjustForPickup
                                : BallMode::kBallRecoveryDown;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.camera_request = smoothed_.visible ? CameraRequest::kNone
                                                : CameraRequest::kDown;
      result.command = smoothed_.visible
                           ? ComputeFineAdjustPlaceholderCommand()
                           : ComputeRecoveryCommand();
      last_command_ = result.command;
    } else if (now_sec - state_enter_sec_ + kTimeEpsilon >=
               std::max(0.0, config_.camera_motion_timeout_sec)) {
      // 카메라 하향 요청은 유지하되, 구동기가 완료 상태를 보내지 않을 때 로봇이
      // 무한히 걷지 않도록 이동 명령을 정지한다.
      result.command = {};
      last_command_ = {};
    }
    return result;
  case BallMode::kFineAdjustForPickup:
    result.active = true;
    result.command = ComputeFineAdjustPlaceholderCommand();
    last_command_ = result.command;
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.fine_adjust_placeholder_duration_sec)) {
      mode_ = BallMode::kPickupBall;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
      result.reached_pickup_pose = true;
      result.command = {};
      last_command_ = {};
    }
    return result;
  case BallMode::kPickupBall:
    result.active = true;
    result.reached_pickup_pose = true;
    result.command = {};
    last_command_ = {};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.pickup_placeholder_duration_sec)) {
      mode_ = BallMode::kVerifyPickup;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
    }
    return result;
  case BallMode::kVerifyPickup:
    // 향후 손/팔 상태와 탑뷰 재검출 결과로 실제 집기 성공 여부를 확인한다.
    // 현재는 정지한 채 임시 시간만 지난 뒤 다음 단계로 넘긴다.
    result.active = true;
    result.reached_pickup_pose = true;
    result.command = {};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.pickup_verification_placeholder_sec)) {
      mode_ = BallMode::kStandUpAfterPickup;
      state_enter_sec_ = now_sec;
      result.mode = mode_;
    }
    return result;
  case BallMode::kStandUpAfterPickup:
    // 향후 일어나기 모션 완료 피드백을 기다리는 상태다.
    result.active = true;
    result.command = {};
    if (now_sec - state_enter_sec_ + kTimeEpsilon >=
        std::max(0.0, config_.stand_up_placeholder_sec)) {
      mode_ = BallMode::kReturnCameraToLine;
      state_enter_sec_ = now_sec;
      ClearTrackingState();
      result.mode = mode_;
      result.camera_request = CameraRequest::kForward;
      result.tracked = {};
    }
    return result;
  case BallMode::kReturnCameraToLine:
    result.active = true;
    result.camera_request = CameraRequest::kForward;
    result.command = {};
    if (camera_feedback.actual_mode == CameraMode::kForward &&
        camera_feedback.settled) {
      // 공 무시 시간은 점선 추종 재개 후의 시간을 뜻하므로, 카메라가 실제로 정면에
      // 복귀하고 안정화된 뒤부터 측정한다.
      ignore_ball_until_sec_ =
          now_sec + std::max(0.0, config_.ball_ignore_duration_sec);
      mode_ = BallMode::kLineFollow;
      state_enter_sec_ = now_sec;
      // 카메라 정면 복귀가 확인된 프레임에서 받은 정상 점선 기준속도는 보존한다.
      // 공 추적기와 고정값들은 비운 상태로 공 무시 시간을 시작한다.
      ClearTrackingState(false);
      result = {};
      result.mode = BallMode::kLineFollow;
    }
    return result;
  }
  return result;
}

void BallController::UpdateTracker(
    const std::optional<ObjectTarget> &ball_target, int image_width,
    int image_height) {
  const bool image_valid = image_width > 1 && image_height > 1;
  const bool detected = ball_target.has_value() && image_valid;
  bool tilt_condition_met = false;
  hit_history_.push_back(detected);
  while (static_cast<int>(hit_history_.size()) >
         std::max(1, config_.stable_window)) {
    hit_history_.pop_front();
  }
  if (detected) {
    lost_count_ = 0;
    TrackedBall observed;
    observed.visible = true;
    observed.center_px = ball_target->center_rectified
                             ? ball_target->rectified_center_px
                             : ball_target->center_px;
    observed.u_norm = Clamp(observed.center_px.u / SafeDenominator(image_width), 0.0, 1.0);
    observed.v_norm = Clamp(observed.center_px.v / SafeDenominator(image_height), 0.0, 1.0);
    // 카메라 하향 조건은 보정 전 원본 영상 좌표를 기준으로 한다. IMU 좌표 보정은
    // 조향에는 사용하지만, 화면 높이 75%라는 하향 기준 자체를 바꾸면 안 된다.
    const double raw_v_norm = Clamp(
        ball_target->center_px.v / SafeDenominator(image_height), 0.0, 1.0);
    last_seen_u_norm_ = observed.u_norm;
    tilt_condition_met = raw_v_norm >= config_.tilt_down_v_norm;
    observed.h_norm = Clamp(ball_target->height_px / SafeDenominator(image_height), 0.0, 1.0);
    observed.area_norm = Clamp(ball_target->area_px /
                                   SafeDenominator(static_cast<double>(image_width) * image_height),
                               0.0, 1.0);
    observed.confidence = ball_target->confidence;
    const double alpha = Clamp(config_.smooth_alpha, 0.0, 1.0);
    if (!has_smoothed_) {
      smoothed_ = observed;
      has_smoothed_ = true;
    } else {
      smoothed_.visible = true;
      smoothed_.center_px.u = (1.0 - alpha) * smoothed_.center_px.u + alpha * observed.center_px.u;
      smoothed_.center_px.v = (1.0 - alpha) * smoothed_.center_px.v + alpha * observed.center_px.v;
      smoothed_.u_norm = (1.0 - alpha) * smoothed_.u_norm + alpha * observed.u_norm;
      smoothed_.v_norm = (1.0 - alpha) * smoothed_.v_norm + alpha * observed.v_norm;
      smoothed_.h_norm = (1.0 - alpha) * smoothed_.h_norm + alpha * observed.h_norm;
      smoothed_.area_norm = (1.0 - alpha) * smoothed_.area_norm + alpha * observed.area_norm;
      smoothed_.confidence = observed.confidence;
    }
  } else {
    ++lost_count_;
    smoothed_.visible = false;
  }
  tilt_trigger_history_.push_back(tilt_condition_met);
  while (static_cast<int>(tilt_trigger_history_.size()) >
         std::max(1, config_.tilt_down_window)) {
    tilt_trigger_history_.pop_front();
  }
  const int hits = static_cast<int>(
      std::count(hit_history_.begin(), hit_history_.end(), true));
  smoothed_.stable = hits >= std::max(1, config_.stable_min_hits);
  const bool upper_acquire_condition =
      detected &&
      Clamp(ball_target->center_px.v / SafeDenominator(image_height),
            0.0, 1.0) < config_.upper_acquire_v_norm;
  upper_acquire_history_.push_back(upper_acquire_condition);
  while (static_cast<int>(upper_acquire_history_.size()) >
         std::max(1, config_.stable_window)) {
    upper_acquire_history_.pop_front();
  }
}

MotionCommand BallController::ComputeFarCommand(const TrackedBall &ball) const {
  const double u_error = (ball.u_norm - config_.far_u_des_norm) / 0.5;
  const double wz_raw = -config_.far_wz_max *
                        std::tanh(config_.far_heading_gain * u_error);
  MotionCommand command;
  // 사용자가 정한 감속 비율만 원거리 접근 vx에 적용한다. 점선을 놓친 뒤에도 양수가
  // 될 수 있는 실시간 복구 명령이 아니라, 원거리 공 접근 상태로 전환할 때 고정한
  // 유효 정상 추종속도를 사용한다.
  command.vx = latched_far_line_vx_ *
               Clamp(config_.far_speed_scale, 0.0, 1.0);
  command.wz = LimitRate(last_command_.wz, wz_raw, config_.far_dw_max);
  return command;
}

MotionCommand BallController::ComputeRecoveryCommand() const {
  MotionCommand command;
  const double horizontal_error = last_seen_u_norm_ - 0.50;
  if (std::abs(horizontal_error) <=
      std::max(0.0, config_.recovery_center_tolerance_norm)) {
    command.vx = std::max(0.0, config_.recovery_forward_vx);
  } else {
    command.wz = horizontal_error > 0.0
                     ? -std::abs(config_.recovery_turn_wz)
                     : std::abs(config_.recovery_turn_wz);
  }
  return command;
}

MotionCommand BallController::ComputeTiltCommand(double /*현재_시간_초*/) const {
  MotionCommand command;
  command.vx = latched_tilt_vx_;
  // 카메라가 움직이는 동안에는 공 위치에 대해 의도적으로 개방루프 제어를 사용한다.
  // 시야가 회전하면서 움직이는 bbox를 쫓지 않고, 미리 맞춘 방향을 유지하며 직진한다.
  command.wz = 0.0;
  return command;
}

MotionCommand BallController::ComputeFineAdjustPlaceholderCommand() const {
  MotionCommand command;
  command.vx = std::max(0.0, config_.fine_adjust_placeholder_vx);
  return command;
}

void BallController::PushRecentCommand(const MotionCommand &command) {
  recent_commands_.push_back(command);
  while (static_cast<int>(recent_commands_.size()) >
         std::max(1, config_.hold_cmd_window)) {
    recent_commands_.pop_front();
  }
}

void BallController::ClearTrackingState(bool clear_line_reference) {
  lost_count_ = 0;
  hit_history_.clear();
  upper_acquire_history_.clear();
  has_smoothed_ = false;
  smoothed_ = {};
  recent_commands_.clear();
  last_command_ = {};
  tilt_trigger_history_.clear();
  recovery_visible_count_ = 0;
  last_seen_u_norm_ = 0.50;
  if (clear_line_reference) last_tracking_line_vx_ = 0.0;
  latched_far_line_vx_ = 0.0;
  latched_tilt_vx_ = 0.0;
}

void BallController::ResetToLineFollow(bool clear_ignore,
                                       bool clear_line_reference) {
  mode_ = BallMode::kLineFollow;
  state_enter_sec_ = 0.0;
  ClearTrackingState(clear_line_reference);
  if (clear_ignore) ignore_ball_until_sec_ = 0.0;
}

void BallController::Reset() { ResetToLineFollow(true, true); }

} // vision_core 네임스페이스
