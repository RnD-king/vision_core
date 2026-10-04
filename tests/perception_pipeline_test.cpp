#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#include <cassert>
using namespace vision_core;
int main() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_hurdle=c.enable_goal=false;
  MissionController controller(c);
  PerceptionFrameInput in; in.image_width=640; in.image_height=480;
  in.intrinsics={600,600,320,240}; in.command_transport_enabled=true;
  for (int i=0;i<5;++i) {
    PerceptionDetection d;
    d.detection.class_id=c.line_detection.class_id;
    d.detection.confidence=1;
    d.detection.box={310.0,400.0-i*40.0,20.0,20.0};
    in.detections.push_back(d);
  }
  const auto out=controller.StepPerception(in);
  assert(out.perception.raw_line_centers.size()==5);
  assert(out.mission.line_features.guide.valid);
  assert(out.mission.command.action==MissionAction::kStepForwardFive);
  return 0;
}
