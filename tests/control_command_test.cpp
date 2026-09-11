#include <cassert>

#include "vision_core/control_command.hpp"

int main() {
  using namespace vision_core;

  ControlCommandCoordinator coordinator;
  BallResult ball;
  GoalResult goal;
  HurdleResult hurdle;
  const MotionCommand line{0.5, 0.0, 0.1};

  auto command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kVelocity);
  assert(command.mission == MissionType::kLine);
  assert(command.velocity.vx == 0.5);

  hurdle.active = true;
  hurdle.mode = HurdleMode::kRlStopping;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kVelocity);
  assert(command.control_phase == ControlPhase::kRlStopping);
  assert(command.velocity.vx == 0.0);

  hurdle.mode = HurdleMode::kContactWalk;
  hurdle.action_request = HurdleActionRequest::kContactWalk;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kAction);
  assert(command.action == MissionAction::kHurdleContactWalk);
  const auto first_id = command.action_id;
  assert(first_id != 0);

  command = coordinator.Compute(ball, hurdle, goal, line,
                                {first_id, true, false});
  assert(command.command_type == CommandType::kHold);
  assert(command.control_phase == ControlPhase::kWaitingActionDone);
  assert(command.action_id == first_id);

  command = coordinator.Compute(ball, hurdle, goal, line,
                                {first_id, true, true});
  assert(command.command_type == CommandType::kHold);
  assert(command.action_id == 0);

  hurdle.action_request = HurdleActionRequest::kNone;
  hurdle.active = false;
  hurdle.mode = HurdleMode::kLineFollow;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kVelocity);

  hurdle.active = true;
  hurdle.mode = HurdleMode::kContactWalk;
  hurdle.action_request = HurdleActionRequest::kContactWalk;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kAction);
  assert(command.action_id != first_id);

  coordinator.Reset();
  hurdle = {};
  ball.active = true;
  ball.mode = BallMode::kVerifyPickup;
  ball.action_request = BallActionRequest::kVerifyPickup;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kAction);
  assert(command.mission == MissionType::kBall);
  assert(command.mission_phase == static_cast<int>(BallMode::kVerifyPickup));
  assert(command.action == MissionAction::kVerifyPickup);
  assert(command.velocity.vx == 0.0);

  coordinator.Reset();
  ball = {};
  goal.active = true;
  goal.mode = GoalMode::kFineAdjust;
  goal.action_request = GoalActionRequest::kFineAdjustHold;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kAction);
  assert(command.mission == MissionType::kGoal);
  assert(command.action == MissionAction::kFineAdjustHold);
  assert(command.velocity.vx == 0.0);

  coordinator.Reset();
  goal = {};
  ball = {};
  hurdle.active = true;
  hurdle.mode = HurdleMode::kApproach;
  ball.mode = BallMode::kPostPickupLineRecovery;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.command_type == CommandType::kVelocity);
  assert(command.mission == MissionType::kBall);
  assert(command.mission_phase ==
         static_cast<int>(BallMode::kPostPickupLineRecovery));
  assert(command.velocity.vx == 0.0);
  assert(command.velocity.wz == line.wz);

  coordinator.Reset();
  ball = {};
  hurdle = {};
  ball.active = true;
  ball.mode = BallMode::kFineAdjustForPickup;
  ball.action_request = BallActionRequest::kFineAdjustForward;
  hurdle.active = true;
  hurdle.mode = HurdleMode::kContactWalk;
  hurdle.action_request = HurdleActionRequest::kContactWalk;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kHurdle);
  assert(command.action == MissionAction::kHurdleContactWalk);

  // Ball이 먼저 선택되면 Goal/Hurdle이 중간에 활성화돼도 Ball의 명시적인
  // LINE_FOLLOW 이탈 전에는 최종 미션을 바꾸지 않는다.
  coordinator.Reset();
  ball = {};
  hurdle = {};
  goal = {};
  ball.active = true;
  ball.mode = BallMode::kApproachBall;
  ball.command = {0.2, 0.0, 0.1};
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kBall);

  hurdle.active = true;
  hurdle.mode = HurdleMode::kApproach;
  hurdle.command = {0.3, 0.0, -0.1};
  goal.active = true;
  goal.mode = GoalMode::kApproach;
  goal.command = {0.1, 0.0, 0.0};
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kBall);
  assert(command.velocity.vx == 0.2);

  // Ball이 정상 종료된 프레임부터 다음 대기 미션을 선택할 수 있다.
  ball = {};
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kGoal);

  // Goal과 Hurdle도 같은 방식으로 자신의 종료 전까지 잠긴다.
  ball.active = true;
  ball.mode = BallMode::kApproachBall;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kGoal);

  goal = {};
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kHurdle);
  goal.active = true;
  goal.mode = GoalMode::kApproach;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kHurdle);

  coordinator.Reset();
  goal = {};
  hurdle = {};
  ball.has_ball = true;
  ball.mode = BallMode::kStandUpAfterPickup;
  ball.action_request = BallActionRequest::kStandUp;
  command = coordinator.Compute(ball, hurdle, goal, line);
  assert(command.mission == MissionType::kBall);
  assert(command.action == MissionAction::kStandUp);

  // P2P backend는 기존 controller의 연속속도를 보행 ACTION으로 바꾸고,
  // 동일 primitive도 DONE 뒤 새 ID로 다시 실행할 수 있다.
  ControlCommandConfig p2p_config;
  p2p_config.first_action_id = 100;
  p2p_config.locomotion_backend = LocomotionBackend::kP2pAction;
  ControlCommandCoordinator p2p_coordinator(p2p_config);
  ball = {};
  hurdle = {};
  goal = {};
  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0});
  assert(command.command_type == CommandType::kAction);
  assert(command.action_category == ActionCategory::kLocomotion);
  assert(command.action == MissionAction::kWalkForwardSix);
  assert(command.action_id == 100);
  assert(command.velocity.vx == 0.0);

  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0});
  assert(command.command_type == CommandType::kAction);
  assert(command.action_id == 100);

  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0},
                                    {100, true, false});
  assert(command.command_type == CommandType::kHold);
  assert(command.action_category == ActionCategory::kLocomotion);
  assert(command.action_id == 100);

  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0},
                                    {100, true, true});
  assert(command.command_type == CommandType::kAction);
  assert(command.action == MissionAction::kWalkForwardSix);
  assert(command.action_category == ActionCategory::kLocomotion);
  assert(command.action_id == 101);

  // 진행 중 보행은 ID가 완료될 때까지 직렬화한다. 완료된 프레임에는 새
  // 보행보다 controller가 요청한 미션 액션을 우선 발급한다.
  ball.active = true;
  ball.mode = BallMode::kPickupBall;
  ball.action_request = BallActionRequest::kPickup;
  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0});
  assert(command.action_category == ActionCategory::kLocomotion);
  assert(command.action_id == 101);

  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0},
                                    {101, true, true});
  assert(command.command_type == CommandType::kAction);
  assert(command.action_category == ActionCategory::kMission);
  assert(command.action == MissionAction::kPickupBall);
  const auto pickup_id = command.action_id;
  assert(pickup_id == 102);

  // 미션 액션은 DONE 뒤 controller가 요청을 내릴 때까지 같은 상태에서
  // 재발급하지 않는다.
  command = p2p_coordinator.Compute(ball, hurdle, goal,
                                    MotionCommand{0.30, 0.0, 0.0},
                                    {pickup_id, true, true});
  assert(command.command_type == CommandType::kHold);
  assert(command.action_id == 0);
  assert(command.action_category == ActionCategory::kNone);

  ball.action_request = BallActionRequest::kNone;
  ball.mode = BallMode::kApproachBall;
  ball.command = {0.30, 0.0, -0.20};
  command = p2p_coordinator.Compute(ball, hurdle, goal, {});
  assert(command.command_type == CommandType::kAction);
  assert(command.action_category == ActionCategory::kLocomotion);
  assert(command.action == MissionAction::kWalkForwardRightSix);

  // P2P의 영속 정지는 /cmd_vel 대신 새 액션을 만들지 않는 HOLD다.
  ControlCommandCoordinator p2p_idle(p2p_config);
  command = p2p_idle.Compute({}, {}, {}, {});
  assert(command.command_type == CommandType::kHold);
  assert(command.action == MissionAction::kNone);
  assert(command.action_id == 0);
  assert(command.velocity.vx == 0.0);
  return 0;
}
