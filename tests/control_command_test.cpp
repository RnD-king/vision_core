#include "vision_core/control_command.hpp"
#include <cassert>
using namespace vision_core;
int main() {
  ControlCommandCoordinator c({1,10});
  ActionRequest first{MissionAction::kStepForwardFive,
                      ActionCategory::kLocomotion,0,true};
  auto out=c.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},0);
  assert(out.command_type==CommandType::kAction && out.action_id==1);
  CommandDeliveryFeedback ack{1,true,false,false};
  out=c.Compute(MissionType::kLine,0,first,CameraRequest::kNone,ack,.1);
  assert(out.command_type==CommandType::kHold);
  ActionRequest next{MissionAction::kStepForwardLeft,
                     ActionCategory::kLocomotion,0,true};
  CommandDeliveryFeedback ready{1,true,false,true};
  out=c.Compute(MissionType::kLine,0,next,CameraRequest::kNone,ready,.2);
  assert(out.command_type==CommandType::kAction && out.action_id==2);
  assert(out.action==MissionAction::kStepForwardLeft);
  CommandDeliveryFeedback queued_ack{2,true,false,false};
  out=c.Compute(MissionType::kLine,0,next,CameraRequest::kNone,queued_ack,.3);
  assert(out.control_phase==ControlPhase::kWaitingQueuedActionStart);
  CommandDeliveryFeedback done{1,true,true,false};
  out=c.Compute(MissionType::kLine,0,{},CameraRequest::kNone,done,.4);
  assert(out.action_id==2 && out.control_phase==ControlPhase::kWaitingActionDone);
  return 0;
}
