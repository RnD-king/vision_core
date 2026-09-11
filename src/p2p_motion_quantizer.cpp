#include "vision_core/p2p_motion_quantizer.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {

P2pMotionQuantizer::P2pMotionQuantizer(const P2pMotionConfig &config)
    : config_(config) {}

LocomotionAction
P2pMotionQuantizer::Quantize(const MotionCommand &command) const {
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
      !std::isfinite(command.wz)) {
    return LocomotionAction::kNone;
  }

  const double forward_deadband = std::max(0.0, config_.forward_deadband);
  const double lateral_deadband = std::max(0.0, config_.lateral_deadband);
  const double yaw_deadband = std::max(0.0, config_.yaw_deadband);
  const double vx = std::abs(command.vx) >= forward_deadband ? command.vx : 0.0;
  const double vy = std::abs(command.vy) >= lateral_deadband ? command.vy : 0.0;
  const double wz = std::abs(command.wz) >= yaw_deadband ? command.wz : 0.0;

  if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
    return LocomotionAction::kNone;
  }

  const double turn_vx_max = std::max(0.0, config_.turn_in_place_vx_max);
  if (wz != 0.0 && std::abs(vx) <= turn_vx_max && vy == 0.0) {
    return wz > 0.0 ? LocomotionAction::kTurnLeftInPlace
                    : LocomotionAction::kTurnRightInPlace;
  }

  const double lateral_ratio = std::max(0.0, config_.lateral_dominance_ratio);
  if (vy != 0.0 && std::abs(vy) > std::abs(vx) * lateral_ratio) {
    return vy > 0.0 ? LocomotionAction::kWalkLeftTwo
                    : LocomotionAction::kWalkRightTwo;
  }

  if (vx < 0.0) return LocomotionAction::kWalkBackwardTwo;

  if (vx > 0.0) {
    const bool long_walk =
        vx >= std::max(forward_deadband, config_.long_forward_vx);
    const bool curved =
        std::abs(wz) >= std::max(yaw_deadband, config_.curve_yaw_threshold);
    if (curved) {
      if (wz > 0.0) {
        return long_walk ? LocomotionAction::kWalkForwardLeftSix
                         : LocomotionAction::kWalkForwardLeftTwo;
      }
      return long_walk ? LocomotionAction::kWalkForwardRightSix
                       : LocomotionAction::kWalkForwardRightTwo;
    }
    return long_walk ? LocomotionAction::kWalkForwardSix
                     : LocomotionAction::kWalkForwardTwo;
  }

  if (vy != 0.0) {
    return vy > 0.0 ? LocomotionAction::kWalkLeftTwo
                    : LocomotionAction::kWalkRightTwo;
  }
  return wz > 0.0 ? LocomotionAction::kTurnLeftInPlace
                  : LocomotionAction::kTurnRightInPlace;
}

MissionAction ToActionCode(LocomotionAction action) {
  return static_cast<MissionAction>(static_cast<std::uint16_t>(action));
}

} // namespace vision_core
