#include "vision_core/control_command.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
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
  ActionRequest ignored_third{MissionAction::kStepForwardRight,
                              ActionCategory::kLocomotion,0,true};
  out=c.Compute(MissionType::kLine,0,ignored_third,CameraRequest::kNone,{},.35);
  assert(out.action_id==2 && out.action==MissionAction::kStepForwardLeft);
  CommandDeliveryFeedback done{1,true,true,false};
  out=c.Compute(MissionType::kLine,0,{},CameraRequest::kNone,done,.4);
  assert(out.action_id==2 && out.control_phase==ControlPhase::kWaitingActionDone);

  ControlCommandCoordinator wrong_id({10,1.0});
  out=wrong_id.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},0);
  assert(out.action_id==10);
  out=wrong_id.Compute(MissionType::kLine,0,first,CameraRequest::kNone,
                       {999,true,true,true},.1);
  assert(out.action_id==10 && out.control_phase==ControlPhase::kWaitingActionAck);

  ControlCommandCoordinator timeout({20,.5});
  out=timeout.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},0);
  out=timeout.Compute(MissionType::kLine,0,first,CameraRequest::kNone,{},.6);
  assert(out.action_id==20);
  assert(out.command_type==CommandType::kHold);
  assert(out.control_phase==ControlPhase::kActionAckTimedOut);
  out=timeout.Compute(MissionType::kLine,0,first,CameraRequest::kNone,
                      {20,true,false,false},.7);
  assert(out.control_phase==ControlPhase::kActionAckTimedOut);
  return 0;
}
