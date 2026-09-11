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
  std::uint64_t first_action_id{1};
  // 기본값은 기존 동작과 동일한 연속속도 출력이다.
  LocomotionBackend locomotion_backend{LocomotionBackend::kVelocity};
  P2pMotionConfig p2p;
};

// 각 미션 controller의 후보를 하나의 명령으로 합치고, ACTION은 ACK 전까지
// 같은 ID로 반복하며 ACK 뒤에는 DONE까지 HOLD한다.
class ControlCommandCoordinator {
public:
  explicit ControlCommandCoordinator(
      const ControlCommandConfig &config = ControlCommandConfig{});

  ControlCommand Compute(const BallResult &ball_result,
                         const HurdleResult &hurdle_result,
                         const GoalResult &goal_result,
                         const MotionCommand &line_candidate,
                         const CommandDeliveryFeedback &feedback = {});
  void Reset();

private:
  ControlCommand BeginAction(ControlCommand command, MissionAction action,
                             ActionCategory category);

  ControlCommandConfig config_;
  P2pMotionQuantizer p2p_quantizer_;
  // 새 물체 미션은 kLine에서만 선택한다. 선택된 미션은 해당 controller가
  // 명시적으로 LINE_FOLLOW로 돌아올 때까지 다른 검출로 교체하지 않는다.
  MissionType active_mission_{MissionType::kLine};
  std::uint64_t next_action_id_{1};
  std::uint64_t pending_action_id_{0};
  MissionAction pending_action_{MissionAction::kNone};
  ActionCategory pending_action_category_{ActionCategory::kNone};
  bool pending_acknowledged_{false};
  // Mission 액션만 controller가 request를 내렸다가 해제할 때까지 재실행을
  // 억제한다. Locomotion 액션은 DONE 뒤 같은 종류도 새 ID로 반복 가능하다.
  MissionAction suppressed_mission_action_{MissionAction::kNone};
};

} // namespace vision_core
