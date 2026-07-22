#pragma once

#include <optional>
#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

struct ObjectTargetConfig {
  int ball_class_id{1};
  int goal_class_id{2};
  int backboard_class_id{3};
  int hurdle_class_id{4};
  double ball_confidence{0.60};
  double goal_confidence{0.60};
  double backboard_confidence{0.60};
  double hurdle_confidence{0.60};
  double min_box_width{2.0};
  double min_box_height{2.0};
};

struct ObjectTargets {
  std::optional<ObjectTarget> ball;
  std::optional<ObjectTarget> goal;
  std::optional<ObjectTarget> backboard;
  std::optional<ObjectTarget> hurdle;
};

ObjectTargets ExtractObjectTargets(const std::vector<Detection> &detections,
                                   const ObjectTargetConfig &config);

} // namespace vision_core
