#include <cassert>
#include <limits>

#include "vision_core/ball_controller.hpp"
#include "vision_core/goal_controller.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/p2p_motion_quantizer.hpp"

int main() {
  using namespace vision_core;

  P2pMotionQuantizer quantizer;
  assert(quantizer.Quantize({}) == LocomotionAction::kNone);
  assert(quantizer.Quantize({0.10, 0.0, 0.0}) ==
         LocomotionAction::kWalkForwardTwo);
  assert(quantizer.Quantize({0.30, 0.0, 0.0}) ==
         LocomotionAction::kWalkForwardSix);
  assert(quantizer.Quantize({0.30, 0.0, 0.20}) ==
         LocomotionAction::kWalkForwardLeftSix);
  assert(quantizer.Quantize({0.30, 0.0, -0.20}) ==
         LocomotionAction::kWalkForwardRightSix);
  assert(quantizer.Quantize({0.10, 0.0, 0.20}) ==
         LocomotionAction::kWalkForwardLeftTwo);
  assert(quantizer.Quantize({-0.10, 0.0, 0.0}) ==
         LocomotionAction::kWalkBackwardTwo);
  assert(quantizer.Quantize({0.02, 0.10, 0.0}) ==
         LocomotionAction::kWalkLeftTwo);
  assert(quantizer.Quantize({0.02, -0.10, 0.0}) ==
         LocomotionAction::kWalkRightTwo);
  assert(quantizer.Quantize({0.0, 0.0, 0.20}) ==
         LocomotionAction::kTurnLeftInPlace);
  assert(quantizer.Quantize({0.0, 0.0, -0.20}) ==
         LocomotionAction::kTurnRightInPlace);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  assert(quantizer.Quantize({nan, 0.0, 0.0}) ==
         LocomotionAction::kNone);
  assert(ToActionCode(LocomotionAction::kWalkForwardSix) ==
         MissionAction::kWalkForwardSix);

  assert(SelectP2pMotionProfile(
             MissionType::kLine, 0) == P2pMotionProfile::kNormal);
  assert(SelectP2pMotionProfile(
             MissionType::kBall,
             static_cast<int>(BallMode::kTiltCameraDownAndApproach)) ==
         P2pMotionProfile::kFine);
  assert(SelectP2pMotionProfile(
             MissionType::kBall,
             static_cast<int>(BallMode::kPostPickupLineRecovery)) ==
         P2pMotionProfile::kRecovery);
  assert(SelectP2pMotionProfile(
             MissionType::kHurdle,
             static_cast<int>(HurdleMode::kTiltCameraDownAndSlow)) ==
         P2pMotionProfile::kFine);
  assert(SelectP2pMotionProfile(
             MissionType::kGoal,
             static_cast<int>(GoalMode::kSearch)) ==
         P2pMotionProfile::kRecovery);

  // 같은 속도라도 Normal은 6걸음, Fine/Recovery는 2걸음으로 짧게
  // 재관측한다.
  assert(quantizer.Quantize({0.30, 0.0, 0.0}, MissionType::kLine, 0) ==
         LocomotionAction::kWalkForwardSix);
  assert(quantizer.Quantize({0.10, 0.0, 0.20}, MissionType::kLine, 0) ==
         LocomotionAction::kWalkForwardTwo);
  assert(quantizer.Quantize(
             {0.30, 0.0, 0.0}, MissionType::kBall,
             static_cast<int>(BallMode::kTiltCameraDownAndApproach)) ==
         LocomotionAction::kWalkForwardTwo);
  assert(quantizer.Quantize(
             {0.30, 0.0, 0.0}, MissionType::kGoal,
             static_cast<int>(GoalMode::kSearch)) ==
         LocomotionAction::kWalkForwardTwo);
  return 0;
}
