#pragma once

#include <cstdint>

#include "vision_core/types.hpp"

namespace vision_core {

// P2P 실행기가 제공할 고정 보행 primitive다. 숫자는 외부 ActionCommand의
// action 코드와 일치하며 기존 미션 액션(1~14)과 겹치지 않는다.
enum class LocomotionAction : std::uint16_t {
  kNone = 0,
  kWalkForwardTwo =
      static_cast<std::uint16_t>(MissionAction::kWalkForwardTwo),
  kWalkForwardLeftTwo =
      static_cast<std::uint16_t>(MissionAction::kWalkForwardLeftTwo),
  kWalkForwardRightTwo =
      static_cast<std::uint16_t>(MissionAction::kWalkForwardRightTwo),
  kWalkForwardSix =
      static_cast<std::uint16_t>(MissionAction::kWalkForwardSix),
  kWalkForwardLeftSix =
      static_cast<std::uint16_t>(MissionAction::kWalkForwardLeftSix),
  kWalkForwardRightSix =
      static_cast<std::uint16_t>(MissionAction::kWalkForwardRightSix),
  kWalkBackwardTwo =
      static_cast<std::uint16_t>(MissionAction::kWalkBackwardTwo),
  kWalkLeftTwo = static_cast<std::uint16_t>(MissionAction::kWalkLeftTwo),
  kWalkRightTwo = static_cast<std::uint16_t>(MissionAction::kWalkRightTwo),
  kTurnLeftInPlace =
      static_cast<std::uint16_t>(MissionAction::kTurnLeftInPlace),
  kTurnRightInPlace =
      static_cast<std::uint16_t>(MissionAction::kTurnRightInPlace),
};

struct P2pMotionConfig {
  // 이 값보다 작은 명령축은 0으로 취급한다.
  double forward_deadband{0.02};
  double lateral_deadband{0.02};
  double yaw_deadband{0.05};

  // 전진속도가 이 값 이상이면 2걸음 대신 6걸음 primitive를 선택한다.
  double long_forward_vx{0.22};
  // 전진 중 yaw가 이 값 이상이면 좌/우 곡선 primitive를 선택한다.
  double curve_yaw_threshold{0.10};
  // 전후진이 이 값 이하인 회전 명령은 제자리 회전으로 취급한다.
  double turn_in_place_vx_max{0.05};
  // |vy|가 |vx|의 이 배수보다 크면 전진보다 횡이동을 우선한다.
  double lateral_dominance_ratio{1.0};
};

// 연속 속도 의도를 하나의 고정 보행 primitive로 양자화한다. 상태를 보존하지
// 않으므로 각 primitive DONE 뒤 최신 속도로 다시 호출할 수 있다.
class P2pMotionQuantizer {
public:
  explicit P2pMotionQuantizer(
      const P2pMotionConfig &config = P2pMotionConfig{});

  LocomotionAction Quantize(const MotionCommand &command) const;

private:
  P2pMotionConfig config_;
};

// ControlCommand.action은 기존 ROS/C++ 호환 때문에 MissionAction 타입을 유지한다.
// P2P 보행 코드는 동일한 숫자 영역으로 안전하게 변환한다.
MissionAction ToActionCode(LocomotionAction action);

} // namespace vision_core
