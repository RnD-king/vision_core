#pragma once

#include "vision_core/motion_command_selector.hpp"
#include "vision_core/p2p_motion_quantizer.hpp"

namespace vision_core {

struct CommandDeliveryFeedback {
  std::uint64_t action_id{0};
  bool acknowledged{false};
  bool done{false};
};

enum class LocomotionBackend {
  kVelocity = 0,
  kP2pAction = 1,
};

struct ControlCommandConfig {
  std::uint64_t first_action_id{};
  LocomotionBackend locomotion_backend{};
  P2pMotionConfig p2p;
  P2pMotionConfig p2p_fine;
  P2pMotionConfig p2p_recovery;
};

// 각 미션 controller의 후보를 하나의 명령으로 합친다. P2P에서는 최종 action과
// 양자화 전 pre_p2p_motion을 함께 반환하고, ACTION은 ACK 전까지 같은 ID로
// 반복하며 ACK 뒤에는 DONE까지 HOLD한다. 실행기 어댑터는 상태를 바꾸지 않고
// 같은 action_id의 ACK/DONE만 다음 Compute에 돌려준다.
class ControlCommandCoordinator {
public:
  ControlCommandCoordinator();
  explicit ControlCommandCoordinator(const ControlCommandConfig &config);

  ControlCommand Compute(const BallResult &ball_result,
                         const HurdleResult &hurdle_result,
                         const GoalResult &goal_result,
                         const MotionCommand &line_candidate,
                         const CommandDeliveryFeedback &feedback = {});
  void Reset();

private:
  ControlCommand BeginAction(ControlCommand command, MissionAction action,
                             ActionCategory category,
                             ActionExecutionKind execution_kind,
                             double action_yaw_rad = 0.0);

  ControlCommandConfig config_;
  P2pMotionQuantizer p2p_quantizer_;
  // 새 물체 미션은 kLine에서만 선택한다. 선택된 미션은 해당 controller가
  // 명시적으로 LINE_FOLLOW로 돌아올 때까지 다른 검출로 교체하지 않는다.
  MissionType active_mission_{MissionType::kLine};
  std::uint64_t next_action_id_{1};
  std::uint64_t pending_action_id_{0};
  MissionAction pending_action_{MissionAction::kNone};
  ActionCategory pending_action_category_{ActionCategory::kNone};
  ActionExecutionKind pending_action_execution_kind_{
      ActionExecutionKind::kNone};
  MotionCommand pending_pre_p2p_motion_;
  double pending_action_yaw_rad_{0.0};
  bool pending_acknowledged_{false};
  // 일반 Mission 액션은 controller가 request를 해제할 때까지 재실행을
  // 억제한다. FINE_ADJUST_HOLD와 Locomotion은 DONE 뒤 반복 가능하다.
  MissionAction suppressed_mission_action_{MissionAction::kNone};
};

} // namespace vision_core
