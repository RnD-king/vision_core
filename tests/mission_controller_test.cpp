#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#include <cassert>
using namespace vision_core;
static MissionFrameInput Frame(double now) {
  MissionFrameInput f; f.image_width=640; f.image_height=480; f.now_sec=now;
  f.command_transport_enabled=true;
  f.line_centers={{320,450},{320,400},{320,350},{320,300},{320,250}};
  return f;
}
static LineGuide Guide(double offset) {
  LineGuide g; g.valid=true; g.offset=offset; g.heading_rad=0; return g;
}
int main() {
  auto c=LoadDefaultAlgorithmConfig();
  c.enable_ball=c.enable_hurdle=c.enable_goal=false;
  MissionController controller(c);
  auto in=Frame(0); auto r=controller.Step(in);
  assert(r.command.action==MissionAction::kStepForwardFive);
  assert(r.command.action_id!=0);
  const auto id=r.command.action_id;
  in=Frame(.1); in.delivery_feedback={id,true,false,false};
  r=controller.Step(in); assert(r.command.command_type==CommandType::kHold);
  in=Frame(.2); in.delivery_feedback={id,true,false,true};
  r=controller.Step(in);
  assert(r.command.action_id!=0);
  assert(r.command.action==MissionAction::kStepForwardFive);

  // READY 집계가 invalid여도 moving action 중에 2초 관측 timer를 시작하지
  // 않는다. stationary failure observation은 current DONE 뒤 시작한다.
  controller.Reset();
  in=Frame(0); r=controller.Step(in);
  const auto moving_id=r.command.action_id;
  in=Frame(.1); in.line_centers.clear();
  in.delivery_feedback={moving_id,true,false,true};
  r=controller.Step(in); assert(!r.line_in_recovery);
  in=Frame(5.0); in.line_centers.clear();
  in.delivery_feedback={moving_id,true,true,false};
  r=controller.Step(in); assert(r.line_in_recovery);
  controller.Reset();
  in=Frame(0); in.line_centers.clear(); r=controller.Step(in);
  assert(r.command.command_type==CommandType::kHold);
  assert(r.line_in_recovery);

  // 안정된 마지막 조향 방향만 기억하고, 첫 실패 관측 2초 뒤 같은 방향으로
  // 양수 15도 TURN을 발행한다.
  controller.Reset();
  in=Frame(0); in.line_decision_guide_override=Guide(-.2);
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kStepForwardLeft);
  const auto left_id=r.command.action_id;
  in=Frame(.1); in.line_centers.clear(); in.line_decision_guide_override.reset();
  in.delivery_feedback={left_id,true,true,false};
  r=controller.Step(in); assert(r.command.command_type==CommandType::kHold);
  in=Frame(2.2); in.line_centers.clear();
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kTurnLeft);
  assert(r.command.target_yaw_deg==15);

  // 방향 evidence가 없으면 최초 관측 + 5회 추가 관측 뒤 FINAL HOLD이며,
  // 다음 유효 frame 하나로 자동 재시작하지 않는다.
  controller.Reset();
  in=Frame(0); in.line_centers.clear(); r=controller.Step(in);
  for (int i=1;i<=6;++i) {
    in=Frame(2.1*i); in.line_centers.clear(); r=controller.Step(in);
    assert(r.command.action==MissionAction::kNone);
  }
  in=Frame(13.0); in.line_decision_guide_override=Guide(0);
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kNone);
  assert(r.command.mission_phase==4);
  return 0;
}
