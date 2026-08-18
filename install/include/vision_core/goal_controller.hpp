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
  // 공 복귀 뒤 5초/직선 확인은 외부 mission adapter가 끝낸 뒤 StartAfterPickup을 호출한다.
  double post_pickup_wait_sec{0.0};
  double camera_motion_timeout_sec{3.0};
  // 골대 시야로 카메라를 올리는 동안 라인 방향으로 계속 직진한다.
  double camera_tilt_forward_vx{0.30};
  double search_wz{0.30};
  double target_u_norm{0.50};
  // 골대 앞에서는 항상 저속으로 접근한다.
  double approach_vx{0.25};
  double approach_wz_gain{2.0};
  double approach_wz_max{0.70};
  // 전체 골대로 접근하다가 백보드 깊이가 이 값 이하면 자세 기반 미세정렬로 바꾼다.
  // 목표 백보드 거리 0.50m보다 0.30m 앞에서 미세조정을 시작한다.
  double fine_adjust_start_z_m{0.80};
  // my_cv/ball_and_hoop.py와 같은 투척 위치 계산값이다.
  double hoop_radius_m{0.10};
  double throwing_range_m{0.40};
  double target_yaw_rad{0.0};
  double position_tolerance_m{0.08};
  double yaw_tolerance_rad{0.1396263402};
  double fine_vx_gain{0.60};
  double fine_vy_gain{0.80};
  double fine_wz_gain{1.00};
  // 현재 보행 정책이 거의 반응하지 않는 미소 속도를 피하고, 한 축씩
  // 일정 시간 움직인 뒤 멈춰서 다시 측정한다.
  double fine_translation_min{0.10};
  double fine_wz_min{0.20};
  double fine_vx_max{0.125};
  double fine_vy_max{0.10};
  double fine_wz_max{0.30};
  double fine_pulse_duration_sec{0.80};
  double fine_near_pulse_duration_sec{0.60};
  double fine_near_error_m{0.12};
  double fine_settle_duration_sec{0.60};
  int fine_adjust_window{10};
  int fine_adjust_min_hits{7};
  // 정렬 완료 뒤 실제 슛 모션을 연결하기 전까지 정지하는 시간이다.
  double shoot_placeholder_sec{2.0};
};

// RGB-D 입력부가 백보드 bbox의 좌/우 끝 3x3 깊이 중앙값으로 계산해 전달한다.
// x는 카메라 오른쪽(+), z는 카메라 전방(+), yaw는 우측이 더 가까울 때 (+)다.
struct GoalPoseObservation {
  bool valid{false};
  double x_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  double confidence{0.0};
};

struct TrackedGoalPose {
  bool stable{false};
  bool visible{false};
  double x_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  double confidence{0.0};
};

GoalPoseObservation EstimateGoalPoseFromEdgeDepths(
    double left_u_px, double left_depth_m, double right_u_px,
    double right_depth_m, const Intrinsics &intrinsics,
    double confidence = 1.0);

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
  TrackedGoalPose pose;
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
  GoalResult Compute(const std::optional<ObjectTarget> &goal_target,
                     const std::optional<ObjectTarget> &backboard_target,
                     const GoalPoseObservation &goal_pose,
                     int image_width, int image_height, double now_sec,
                     bool line_reference_valid,
                     const CameraFeedback &camera_feedback);
  static const char *ModeName(GoalMode mode);
  void Reset();

private:
  enum class FineMotionPhase { kReady, kPulse, kSettle };

  static double Clamp(double value, double low, double high);
  void UpdateGoalTracker(const std::optional<ObjectTarget> &target,
                         int image_width, int image_height);
  void UpdatePoseTracker(const std::optional<ObjectTarget> &backboard_target,
                         const GoalPoseObservation &goal_pose);
  MotionCommand ComputeApproachCommand() const;
  MotionCommand ComputeFineAdjustCommand(double now_sec);
  MotionCommand MakeFineAdjustPulse() const;
  bool FineAdjustSettled(double now_sec) const;
  void ResetFineMotion();
  bool PoseReadyForFineAdjust() const;
  bool PoseAligned() const;
  void ClearTracking();

  GoalConfig config_;
  GoalMode mode_{GoalMode::kLineFollow};
  std::deque<bool> hit_history_;
  std::deque<bool> pose_hit_history_;
  std::deque<bool> fine_enter_history_;
  std::deque<bool> fine_adjust_history_;
  int lost_count_{0};
  int pose_lost_count_{0};
  bool has_smoothed_{false};
  bool has_pose_smoothed_{false};
  TrackedGoal tracked_;
  TrackedGoalPose tracked_pose_;
  double state_enter_sec_{0.0};
  FineMotionPhase fine_motion_phase_{FineMotionPhase::kReady};
  MotionCommand fine_pulse_command_;
  double fine_motion_phase_enter_sec_{0.0};
  double fine_pulse_duration_sec_{0.0};
};

} // namespace vision_core
