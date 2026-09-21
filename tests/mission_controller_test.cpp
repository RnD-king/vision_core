#include "vision_core/mission_controller.hpp"
#include "vision_core/config_loader.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

namespace {

using namespace vision_core;

ObjectTarget Target(int class_id, double u_norm, double center_v_norm) {
  ObjectTarget target;
  target.class_id = class_id;
  target.confidence = 1.0;
  target.width_px = 20.0;
  target.height_px = 20.0;
  target.area_px = 400.0;
  target.box_px = {u_norm * 100.0 - 10.0,
                   center_v_norm * 100.0 - 10.0, 20.0, 20.0};
  target.center_px = {u_norm * 100.0, center_v_norm * 100.0};
  target.rectified_center_px = target.center_px;
  return target;
}

MissionFrameInput Frame(double now_sec) {
  MissionFrameInput input;
  input.image_width = 100;
  input.image_height = 100;
  input.now_sec = now_sec;
  input.camera_feedback = {CameraMode::kForward, true};
  input.line_centers = {{50.0, 90.0}, {50.0, 70.0}, {50.0, 50.0},
                        {50.0, 30.0}};
  return input;
}

MissionControllerConfig FastConfig() {
  MissionControllerConfig config = LoadDefaultAlgorithmConfig();
  config.line_features.image_center_u = 50.0;
  config.ball.stable_window = 1;
  config.ball.stable_min_hits = 1;
  config.ball.upper_acquire_v_norm = 0.65;
  config.ball.tilt_down_v_norm = 0.95;
  config.hurdle.stable_window = 1;
  config.hurdle.stable_min_hits = 1;
  config.hurdle.acquire_min_v_norm = 0.60;
  config.hurdle.tilt_trigger_v_norm = 0.95;
  config.goal.stable_window = 1;
  config.goal.stable_min_hits = 1;
  // 진입 프레임에서 POST_PICKUP_WAIT를 관찰하려는 테스트이므로 production
  // 기본값(0초)과 달리 한 프레임 안에 다음 상태로 통과하지 않게 한다.
  config.goal.post_pickup_wait_sec = 1.0;
  return config;
}

void TestOnlyActiveBallControllerAdvances() {
  MissionController controller(FastConfig());
  auto input = Frame(0.0);
  input.ball_target = Target(1, 0.5, 0.5);
  auto result = controller.Step(input);
  assert(result.active_mission == MissionType::kBall);
  assert(result.ball.mode == BallMode::kApproachBall);

  input = Frame(0.1);
  input.ball_target = Target(1, 0.5, 0.5);
  input.hurdle_target = Target(4, 0.5, 0.7);
  input.goal_target = Target(2, 0.5, 0.4);
  result = controller.Step(input);
  assert(result.active_mission == MissionType::kBall);
  assert(result.goal.mode == GoalMode::kLineFollow);
  assert(result.hurdle.mode == HurdleMode::kLineFollow);
  assert(!result.line_computed);
}

void TestHurdleEntryHasPriorityOverBall() {
  MissionController controller(FastConfig());
  auto input = Frame(0.0);
  input.ball_target = Target(1, 0.5, 0.5);
  input.hurdle_target = Target(4, 0.5, 0.7);
  const auto result = controller.Step(input);
  assert(result.active_mission == MissionType::kHurdle);
  assert(result.hurdle.mode == HurdleMode::kApproach);
  assert(result.ball.mode == BallMode::kLineFollow);
}

void TestCarryingBallAllowsOnlyGoalEntry() {
  auto config = FastConfig();
  config.initial_has_ball = true;
  MissionController controller(config);
  auto input = Frame(0.0);
  input.ball_target = Target(1, 0.5, 0.5);
  input.hurdle_target = Target(4, 0.5, 0.7);
  // goal class가 없어도 검증된 backboard만으로 골대 미션에 진입한다.
  input.backboard_target = Target(3, 0.5, 0.4);
  const auto result = controller.Step(input);
  assert(result.active_mission == MissionType::kGoal);
  assert(result.goal.mode == GoalMode::kPostPickupWait);
  assert(result.hurdle.mode == HurdleMode::kLineFollow);
  assert(result.ball.mode == BallMode::kLineFollow);
}

void TestMissionControllerReturnsActionAndPreP2pMotionTogether() {
  auto config = FastConfig();
  config.enable_ball = false;
  config.enable_hurdle = false;
  config.enable_goal = false;
  config.line.line_stable_window = 1;
  config.line.line_stable_min_hits = 1;
  config.command.locomotion_backend = LocomotionBackend::kP2pAction;
  config.command.first_action_id = 500;
  MissionController controller(config);

  auto input = Frame(0.0);
  auto result = controller.Step(input);
  assert(result.command.command_type == CommandType::kAction);
  assert(result.command.action_category == ActionCategory::kLocomotion);
  assert(result.command.action_execution_kind ==
         ActionExecutionKind::kVelocityCompatible);
  assert(result.command.action_id == 500);
  assert(result.command.velocity.vx == 0.0);
  assert(result.command.pre_p2p_motion.vx == result.line_command.vx);
  assert(result.command.pre_p2p_motion.vx > 0.0);
  const MotionCommand first_pre_p2p = result.command.pre_p2p_motion;

  // ACK 뒤에도 같은 action_id와 최초 선택 시점의 PRE-P2P 명령을 유지한다.
  input = Frame(0.1);
  input.delivery_feedback = {500, true, false};
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kHold);
  assert(result.command.control_phase == ControlPhase::kWaitingActionDone);
  assert(result.command.action_id == 500);
  assert(result.command.pre_p2p_motion.vx == first_pre_p2p.vx);

  // 긴 action 후반에는 오른쪽으로 치우친 line을 관측한다. DONE 프레임 자체는
  // 다시 중앙선이지만, 다음 판단은 후반 누적 특징을 사용해야 한다.
  input = Frame(0.2);
  for (auto &point : input.line_centers) point.u = 70.0;
  input.delivery_feedback = {500, false, false};
  result = controller.Step(input);
  assert(result.command.action_id == 500);

  input = Frame(0.3);
  for (auto &point : input.line_centers) point.u = 70.0;
  input.delivery_feedback = {500, false, false};
  result = controller.Step(input);
  assert(result.command.action_id == 500);

  input = Frame(0.4);
  input.delivery_feedback = {500, true, true};
  result = controller.Step(input);
  assert(result.command.command_type == CommandType::kAction);
  assert(result.command.action_id == 501);
  assert(result.command.action == MissionAction::kWalkForwardRightSix);
}

} // namespace

int main() {
  TestOnlyActiveBallControllerAdvances();
  TestHurdleEntryHasPriorityOverBall();
  TestCarryingBallAllowsOnlyGoalEntry();
  TestMissionControllerReturnsActionAndPreP2pMotionTogether();
  std::cout << "mission controller tests passed\n";
  return 0;
}
