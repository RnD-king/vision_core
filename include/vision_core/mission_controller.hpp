#pragma once

#include <optional>
#include <vector>

#include "vision_core/ball_controller.hpp"
#include "vision_core/control_command.hpp"
#include "vision_core/goal_controller.hpp"
#include "vision_core/hurdle_controller.hpp"
#include "vision_core/line_feature_extractor.hpp"
#include "vision_core/line_velocity_controller.hpp"

namespace vision_core {

struct MissionControllerConfig {
  FeatureConfig line_features;
  RuleConfig line;
  BallConfig ball;
  HurdleConfig hurdle;
  GoalConfig goal;
  ControlCommandConfig command;
  double line_observation_dt{1.0 / 15.0};
  bool enable_ball{true};
  bool enable_hurdle{true};
  bool enable_goal{true};
  // Goal 단독 시험처럼 공 집기 과정을 생략하는 구성에서만 사용한다.
  bool initial_has_ball{false};
};

struct MissionFrameInput {
  std::vector<Point2> line_centers;
  double previous_vx{0.0};
  double previous_wz{0.0};
  std::optional<ObjectTarget> ball_target;
  std::optional<ObjectTarget> hurdle_target;
  std::optional<ObjectTarget> goal_target;
  std::optional<ObjectTarget> backboard_target;
  GoalPoseObservation goal_pose;
  int image_width{0};
  int image_height{0};
  double now_sec{0.0};
  CameraFeedback camera_feedback;
  ActionExecutionFeedback action_feedback;
  CommandDeliveryFeedback delivery_feedback;
};

struct MissionFrameResult {
  ControlCommand command;
  MissionType active_mission{MissionType::kLine};
  Features line_features;
  MotionCommand line_command;
  bool line_computed{false};
  bool line_in_recovery{true};
  bool has_ball{false};
  BallResult ball;
  HurdleResult hurdle;
  GoalResult goal;
};

// 전체 미션의 유일한 진입점이다. LINE에서만 진입 후보를 관찰하고, 물체
// 미션이 잠긴 동안에는 해당 controller 하나만 실행한다.
class MissionController {
public:
  explicit MissionController(
      const MissionControllerConfig &config = MissionControllerConfig{});

  MissionFrameResult Step(const MissionFrameInput &input);
  MissionType ActiveMission() const { return active_mission_; }
  bool HasBall() const { return has_ball_; }
  void Reset();

private:
  MotionCommand StepLine(const Features &features, bool *reference_valid,
                         Features *updated_features);
  void BeginLineReacquisition();
  void EnterMission(MissionType mission);
  void FinishMission();

  MissionControllerConfig config_;
  LineVelocityController line_controller_;
  BallController ball_controller_;
  HurdleController hurdle_controller_;
  GoalController goal_controller_;
  ControlCommandCoordinator command_coordinator_;
  LineFeatureState line_feature_state_;
  MissionType active_mission_{MissionType::kLine};
  bool has_ball_{false};
  BallResult ball_result_;
  HurdleResult hurdle_result_;
  GoalResult goal_result_;
};

} // namespace vision_core
