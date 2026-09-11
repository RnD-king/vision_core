#include "vision_core/mission_controller.hpp"

namespace vision_core {

MissionController::MissionController(const MissionControllerConfig &config)
    : config_(config),
      line_controller_(config.line, config.line_observation_dt),
      ball_controller_(config.ball), hurdle_controller_(config.hurdle),
      goal_controller_(config.goal), command_coordinator_(config.command),
      has_ball_(config.initial_has_ball) {
  if (has_ball_) goal_controller_.StartAfterPickup(0.0);
}

MotionCommand MissionController::StepLine(const Features &features,
                                          bool *reference_valid,
                                          Features *updated_features) {
  line_controller_.Observe(features, {});
  Features current = features;
  current.in_recovery = line_controller_.InRecovery() ? 1.0 : 0.0;
  const Command command = line_controller_.Compute(current);
  if (reference_valid != nullptr) {
    *reference_valid = !line_controller_.InRecovery();
  }
  if (updated_features != nullptr) {
    current.in_recovery = line_controller_.InRecovery() ? 1.0 : 0.0;
    *updated_features = current;
  }
  return {command.vx, 0.0, command.wz};
}

void MissionController::BeginLineReacquisition() {
  line_controller_.BeginReacquisition();
}

void MissionController::EnterMission(MissionType mission) {
  active_mission_ = mission;
  if (mission == MissionType::kBall) {
    hurdle_controller_.Reset();
    goal_controller_.Reset();
    hurdle_result_ = {};
    goal_result_ = {};
  } else if (mission == MissionType::kHurdle) {
    ball_controller_.Reset();
    goal_controller_.Reset();
    ball_result_ = {};
    goal_result_ = {};
    has_ball_ = false;
  } else if (mission == MissionType::kGoal) {
    hurdle_controller_.Reset();
    hurdle_result_ = {};
  }
}

void MissionController::FinishMission() {
  active_mission_ = MissionType::kLine;
  ball_result_ = {};
  hurdle_result_ = {};
  goal_result_ = {};
}

MissionFrameResult MissionController::Step(const MissionFrameInput &input) {
  MissionFrameResult output;
  bool line_reference_valid = false;

  const auto compute_line = [&]() {
    const Features features = ComputeLineFeatures(
        input.line_centers, input.image_width, input.image_height,
        line_controller_.InRecovery(), input.previous_vx, input.previous_wz,
        config_.line_features, &line_feature_state_);
    output.line_command = StepLine(features, &line_reference_valid,
                                   &output.line_features);
    output.line_computed = true;
  };

  switch (active_mission_) {
  case MissionType::kLine: {
    compute_line();
    ball_result_ = {};
    hurdle_result_ = {};
    goal_result_ = {};

    if (has_ball_) {
      if (config_.enable_goal) {
        BallResult carrying;
        carrying.has_ball = true;
        carrying.mode = BallMode::kLineFollow;
        goal_controller_.UpdateBallState(carrying);
        goal_result_ = goal_controller_.Compute(
            input.goal_target, input.backboard_target, input.goal_pose,
            input.image_width, input.image_height, input.now_sec,
            line_reference_valid, input.camera_feedback,
            input.action_feedback);
        if (goal_result_.mode != GoalMode::kLineFollow) {
          EnterMission(MissionType::kGoal);
        }
      }
    } else {
      if (config_.enable_hurdle) {
        hurdle_result_ = hurdle_controller_.Compute(
            input.hurdle_target, input.image_width, input.image_height,
            input.now_sec, output.line_command.vx, line_reference_valid,
            input.camera_feedback, input.action_feedback);
        if (hurdle_result_.mode != HurdleMode::kLineFollow) {
          EnterMission(MissionType::kHurdle);
          break;
        }
      }

      // 허들이 한 프레임이라도 현재 보이는 동안에는 안전 우선순위상 Ball
      // 진입 이력을 진행하지 않는다.
      if (config_.enable_ball && config_.enable_hurdle &&
          input.hurdle_target.has_value()) {
        ball_controller_.ClearEntryEvidence();
      } else if (config_.enable_ball) {
        ball_result_ = ball_controller_.Compute(
            input.ball_target, input.image_width, input.image_height,
            input.now_sec, output.line_command.vx, line_reference_valid,
            input.camera_feedback, input.action_feedback);
        if (ball_result_.mode != BallMode::kLineFollow) {
          EnterMission(MissionType::kBall);
        }
      }
    }
    break;
  }
  case MissionType::kBall: {
    const BallMode previous_mode = ball_result_.mode;
    if (previous_mode == BallMode::kPostPickupLineRecovery) compute_line();
    ball_result_ = ball_controller_.Compute(
        input.ball_target, input.image_width, input.image_height,
        input.now_sec, output.line_command.vx, line_reference_valid,
        input.camera_feedback, input.action_feedback);
    has_ball_ = ball_result_.has_ball;
    if (previous_mode != BallMode::kPostPickupLineRecovery &&
        ball_result_.mode == BallMode::kPostPickupLineRecovery) {
      BeginLineReacquisition();
    }
    if (ball_result_.mode == BallMode::kLineFollow && !ball_result_.active) {
      if (!output.line_computed) BeginLineReacquisition();
      if (has_ball_ && config_.enable_goal) {
        goal_controller_.UpdateBallState(ball_result_);
      }
      FinishMission();
    }
    break;
  }
  case MissionType::kHurdle: {
    hurdle_result_ = hurdle_controller_.Compute(
        input.hurdle_target, input.image_width, input.image_height,
        input.now_sec, 0.0, false, input.camera_feedback,
        input.action_feedback);
    if (hurdle_result_.mode == HurdleMode::kLineFollow &&
        !hurdle_result_.active) {
      BeginLineReacquisition();
      FinishMission();
    }
    break;
  }
  case MissionType::kGoal: {
    const GoalMode previous_mode = goal_result_.mode;
    if (previous_mode == GoalMode::kHeadingRecovery) compute_line();
    goal_result_ = goal_controller_.Compute(
        input.goal_target, input.backboard_target, input.goal_pose,
        input.image_width, input.image_height, input.now_sec,
        line_reference_valid, input.camera_feedback, input.action_feedback);
    has_ball_ = goal_controller_.HasBall();
    ball_controller_.SetHasBall(has_ball_);
    if (previous_mode != GoalMode::kHeadingRecovery &&
        goal_result_.mode == GoalMode::kHeadingRecovery) {
      BeginLineReacquisition();
    }
    if (goal_result_.mode == GoalMode::kLineFollow && !goal_result_.active) {
      if (!output.line_computed) BeginLineReacquisition();
      FinishMission();
    }
    break;
  }
  default:
    active_mission_ = MissionType::kLine;
    compute_line();
    break;
  }

  output.command = command_coordinator_.Compute(
      ball_result_, hurdle_result_, goal_result_, output.line_command,
      input.delivery_feedback);
  output.active_mission = output.command.mission;
  output.ball = ball_result_;
  output.hurdle = hurdle_result_;
  output.goal = goal_result_;
  output.has_ball = has_ball_;
  output.line_in_recovery = line_controller_.InRecovery();
  output.line_features.in_recovery = output.line_in_recovery ? 1.0 : 0.0;
  return output;
}

void MissionController::Reset() {
  active_mission_ = MissionType::kLine;
  has_ball_ = config_.initial_has_ball;
  line_controller_.Reset();
  ball_controller_.Reset();
  hurdle_controller_.Reset();
  goal_controller_.Reset();
  command_coordinator_.Reset();
  line_feature_state_.Reset();
  ball_result_ = {};
  hurdle_result_ = {};
  goal_result_ = {};
  if (has_ball_) goal_controller_.StartAfterPickup(0.0);
}

} // namespace vision_core
