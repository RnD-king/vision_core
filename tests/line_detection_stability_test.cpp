#include "vision_core/line_velocity_controller.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

namespace {

vision_core::Features ReliableLine() {
  vision_core::Features features;
  features.n_visible = 4.0;
  features.u_err_near = 0.0;
  features.u_err_ctrl = 0.0;
  return features;
}

vision_core::Features MissingLine() {
  vision_core::Features features;
  features.n_visible = 0.0;
  features.in_recovery = 1.0;
  return features;
}

void TestLineUsesTenFrameSevenHitRule() {
  vision_core::LineVelocityController controller;
  const auto reliable = ReliableLine();
  for (int frame = 0; frame < 6; ++frame) {
    controller.Observe(reliable, {});
    assert(controller.InRecovery());
  }
  controller.Observe(reliable, {});
  assert(!controller.InRecovery());

  controller.BeginReacquisition();
  assert(controller.InRecovery());
  for (int frame = 0; frame < 6; ++frame) {
    controller.Observe(reliable, {});
    assert(controller.InRecovery());
  }
  controller.Observe(reliable, {});
  assert(!controller.InRecovery());

  const auto missing = MissingLine();
  for (int frame = 0; frame < 3; ++frame) {
    controller.Observe(missing, {});
    assert(!controller.InRecovery());
  }
  controller.Observe(missing, {});
  assert(controller.InRecovery());
}

void TestVisibleRecoveryUsesCurrentLookahead() {
  vision_core::LineVelocityController controller;
  auto features = ReliableLine();
  features.u_err_ctrl = 0.5;

  controller.BeginReacquisition();
  const auto tracking_command = controller.Compute(features);

  assert(controller.InRecovery());
  assert(tracking_command.wz < 0.0);
}

void TestLowVisibilityRecoveryUsesPathMemory() {
  vision_core::LineVelocityController controller;
  auto features = ReliableLine();
  features.n_visible = 1.0;
  features.u_err_ctrl = 0.5;

  controller.BeginReacquisition();
  const auto command = controller.Compute(features);

  assert(controller.InRecovery());
  assert(command.wz > 0.0);
}

void TestSearchRotationDoesNotWalkForward() {
  vision_core::LineVelocityController controller;
  controller.BeginReacquisition();
  const auto command = controller.ComputeSearchRotation();
  assert(command.vx == 0.0);
  assert(command.wz != 0.0);
}

void TestLookaheadApproachUsesOnlyFarPoint() {
  vision_core::LineVelocityController controller;
  auto features = ReliableLine();
  features.u_err_near = -0.9;
  features.u_err_lookahead = 0.4;
  features.u_err_ctrl = -0.7;
  features.slope = -1.0;
  const auto command = controller.ComputeLookaheadApproach(features);
  assert(command.vx > 0.0 && command.vx <= 0.45);
  assert(command.wz < 0.0);
}

} // namespace

int main() {
  TestLineUsesTenFrameSevenHitRule();
  TestVisibleRecoveryUsesCurrentLookahead();
  TestLowVisibilityRecoveryUsesPathMemory();
  TestSearchRotationDoesNotWalkForward();
  TestLookaheadApproachUsesOnlyFarPoint();
  std::cout << "line detection stability tests passed\n";
  return 0;
}
