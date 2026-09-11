#include <cassert>
#include <limits>

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
  return 0;
}
