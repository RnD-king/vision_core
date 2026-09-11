#pragma once

#include <cstdint>

namespace vision_core {

struct Point2 {
  double u{0.0};
  double v{0.0};
};

struct Intrinsics {
  double fx{600.0};
  double fy{600.0};
  double cx{320.0};
  double cy{240.0};
};

struct FeatureConfig {
  int max_centers{8};
  double image_center_u{320.0};
  double lookahead_delta_v_px{220.0};
  // 점이 충분할 때 화면 아래/위의 겹치는 지역 직선 방향 차이로 커브를
  // 추정한다. 커브가 강할수록 lookahead 거리를 기본값의 68%까지 줄인다.
  int curve_min_points{5};
  double curve_min_v_span_px{80.0};
  int curve_local_fit_points{4};
  double curve_full_scale_angle_rad{0.35};
  double curve_smoothing_alpha{0.20};
  double curve_missing_decay{0.96};
  double curve_lookahead_min_scale{0.68};
  // 정상 추종은 가까운 점 35%, 먼 lookahead점 65%를 섞어 코너 안쪽 절단을 줄인다.
  double lookahead_alpha_normal{0.65};
  double lookahead_alpha_recovery{0.85};
  double recover_enter_nvis{2.0};
  double recover_exit_nvis{3.0};
  double recover_enter_u{0.70};
  double recover_exit_u{0.35};
};

struct LineFeatureState {
  double filtered_curve_score{0.0};
  bool initialized{false};

  void Reset() {
    filtered_curve_score = 0.0;
    initialized = false;
  }
};

struct Features {
  double u_err_near{0.0};
  double u_err_lookahead{0.0};
  double u_err_ctrl{0.0};
  double slope{0.0};
  double n_visible{0.0};
  double in_recovery{0.0};
  double vx_prev{0.0};
  double wz_prev{0.0};
};

struct RuleConfig {
  int line_stable_window{10};
  int line_stable_min_hits{7};
  double line_reacquire_nvis{3.0};
  double line_reacquire_u{0.35};
  double cmd_vx_min{0.10};
  double cmd_vx_max{1.20};
  double cmd_wz_min{-1.90};
  double cmd_wz_max{1.90};
  double v_base{0.85};
  // 정상 라인 추종에서 계산된 vx 전체를 기존의 75%로 낮춘다.
  double tracking_speed_scale{0.80};
  double k_u{3.00};
  double k_slope{3.40};
  double k_v_u{0.35};
  double k_v_slope{0.35};
  double dv_max{0.12};
  double dw_max{0.40};
  double recover_vx{0.12};
  double recover_wz{0.75};
  double line_recovery_vx_max{0.45};
  double low_visible_n{2.0};
  double no_visible_n{0.5};
  double low_visible_vx{0.18};
  double no_visible_vx{0.10};
  double low_visible_wz_decay{0.90};
  double no_visible_wz_decay{0.95};

  double recover_coast_s{0.30};
  double recover_lookahead_m{0.55};
  double recover_search_delay_s{0.35};
  double recover_sweep_period_s{1.20};
  double recover_search_wz_min{0.22};
  double recover_search_wz_max{0.70};
  double recover_k_bearing{1.35};
  double recover_k_heading{0.45};
  double recover_k_cross_track{0.70};
  double recover_path_backtrack_m{0.35};
  double recover_path_forward_margin_m{1.00};
  double recover_side_memory_alpha{0.18};
};

struct Pose2 {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Command {
  double vx{0.0};
  double wz{0.0};
};

struct Box2 {
  double x{0.0};
  double y{0.0};
  double width{0.0};
  double height{0.0};
};

struct Detection {
  Box2 box;
  double confidence{0.0};
  int class_id{0};
};

struct ObjectTarget {
  int class_id{0};
  double confidence{0.0};
  Box2 box_px;
  Point2 center_px;
  Point2 rectified_center_px;
  double width_px{0.0};
  double height_px{0.0};
  double area_px{0.0};
  bool center_rectified{false};
};

struct MotionCommand {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

// 외부 실행기와 공유하는 통합 명령 분류다. 숫자값은 C/ROS 연결층에서도
// 그대로 사용하므로 기존 값의 의미를 변경하지 않는다.
enum class CommandType {
  kNone = 0,
  kVelocity = 1,
  kAction = 2,
  kHold = 3,
};

enum class MissionType {
  kNone = 0,
  kLine = 1,
  kBall = 2,
  kGoal = 3,
  kHurdle = 4,
};

enum class MissionAction {
  kNone = 0,
  kStepForwardHalf = 1,
  kStepForward = 2,
  kStepBackward = 3,
  kStepLeft = 4,
  kStepRight = 5,
  kTurnLeft = 6,
  kTurnRight = 7,
  kPickupBall = 8,
  kStandUp = 9,
  kHurdleContactWalk = 10,
  kCrossHurdle = 11,
  kShoot = 12,
  kVerifyPickup = 13,
  kFineAdjustHold = 14,
  // P2P locomotion backend가 사용하는 고정 보행 코드. 기존 1~14의 값과
  // 의미는 ROS/C API 호환을 위해 그대로 유지한다.
  kWalkForwardTwo = 15,
  kWalkForwardLeftTwo = 16,
  kWalkForwardRightTwo = 17,
  kWalkForwardSix = 18,
  kWalkForwardLeftSix = 19,
  kWalkForwardRightSix = 20,
  kWalkBackwardTwo = 21,
  kWalkLeftTwo = 22,
  kWalkRightTwo = 23,
  kTurnLeftInPlace = 24,
  kTurnRightInPlace = 25,
};

// action 숫자는 하나의 ROS 토픽으로 전달하지만, 생성 원인과 DONE 처리 규칙은
// 분리한다. Mission은 controller 상태 전이용 단발 동작이고 Locomotion은
// 연속 속도를 양자화한 반복 가능한 보행 블록이다.
enum class ActionCategory {
  kNone = 0,
  kMission = 1,
  kLocomotion = 2,
};

enum class ControlPhase {
  kMission = 0,
  kRlStopping = 1,
  kWaitingActionAck = 2,
  kWaitingActionDone = 3,
};

enum class CameraMode {
  kForward = 0,
  kDown = 1,
  kTransition = 2,
  kGoal = 3,
};

enum class CameraRequest {
  kNone = 0,
  kDown = 1,
  kForward = 2,
  kGoal = 3,
};

struct CameraFeedback {
  CameraMode actual_mode{CameraMode::kForward};
  bool settled{true};
};

// enabled=false이면 기존 고정시간 placeholder를 사용한다. ROS 연결층처럼
// enabled=true인 호출자는 DONE이 들어온 프레임에만 action_done=true로 전달한다.
struct ActionExecutionFeedback {
  bool enabled{false};
  bool action_done{false};
  bool action_active{false};
};

struct ControlCommand {
  CommandType command_type{CommandType::kVelocity};
  MissionType mission{MissionType::kLine};
  int mission_phase{0};
  ControlPhase control_phase{ControlPhase::kMission};
  MotionCommand velocity;
  MissionAction action{MissionAction::kNone};
  ActionCategory action_category{ActionCategory::kNone};
  std::uint64_t action_id{0};
  CameraRequest camera_request{CameraRequest::kNone};
};

} // namespace vision_core
