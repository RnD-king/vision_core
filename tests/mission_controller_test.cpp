#include "vision_core/config_loader.hpp"
#include "vision_core/mission_controller.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
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
static void ShiftLine(MissionFrameInput &frame, double u) {
  for (auto &point : frame.line_centers) point.u = u;
}
static ObjectTarget Ball(double u_norm, double v_norm) {
  ObjectTarget target;
  target.center_px = {u_norm * 640.0, v_norm * 480.0};
  target.width_px = 20.0;
  target.height_px = 20.0;
  target.area_px = 400.0;
  target.confidence = 1.0;
  return target;
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

  // ACK 전 정지 관측은 long-action 후반 accumulator에 포함하지 않는다.
  // ACK 뒤 관측은 모두 straight이므로 READY 예약도 straight여야 한다.
  controller.Reset();
  in=Frame(0); r=controller.Step(in);
  const auto delayed_ack_id=r.command.action_id;
  in=Frame(.1); ShiftLine(in, 500); r=controller.Step(in);
  in=Frame(.7); ShiftLine(in, 500); r=controller.Step(in);
  in=Frame(.8); in.delivery_feedback={delayed_ack_id,true,false,false};
  r=controller.Step(in);
  in=Frame(1.0); in.delivery_feedback={delayed_ack_id,true,false,true};
  r=controller.Step(in);
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

  // 정상 RIGHT 뒤 READY의 reliable STRAIGHT 판단은 과거 방향 evidence를
  // 지운다. 이후 line loss는 TURN_RIGHT가 아니라 stationary retry다.
  controller.Reset();
  in=Frame(0); in.line_decision_guide_override=Guide(.3);
  r=controller.Step(in);
  const auto right_id=r.command.action_id;
  in=Frame(.1); in.delivery_feedback={right_id,true,false,false};
  r=controller.Step(in);
  in=Frame(1.0); in.delivery_feedback={right_id,true,false,true};
  r=controller.Step(in);
  const auto straight_id=r.command.action_id;
  assert(r.command.action==MissionAction::kStepForwardFive);
  in=Frame(1.1); in.delivery_feedback={straight_id,true,false,false};
  r=controller.Step(in);
  in=Frame(1.2); in.delivery_feedback={right_id,true,true,false};
  r=controller.Step(in);
  in=Frame(1.3); in.line_centers.clear();
  in.delivery_feedback={straight_id,true,true,false};
  r=controller.Step(in);
  in=Frame(3.4); in.line_centers.clear();
  r=controller.Step(in);
  assert(r.command.action==MissionAction::kNone);
  assert(r.command.mission_phase==1);

  // FINAL HOLD에서는 유효한 object detection도 새 mission을 시작하지 않는다.
  auto terminal_config=c;
  terminal_config.line_p2p.failure_observation_sec=0.0;
  terminal_config.line_p2p.no_evidence_max_retries=0;
  terminal_config.enable_ball=true;
  terminal_config.enable_hurdle=false;
  terminal_config.ball.stable_window=1;
  terminal_config.ball.stable_min_hits=1;
  MissionController terminal(terminal_config);
  in=Frame(0); in.line_centers.clear(); r=terminal.Step(in);
  in=Frame(.1); in.line_centers.clear(); r=terminal.Step(in);
  assert(r.command.mission_phase==4);
  in=Frame(.2); in.line_centers.clear(); in.ball_target=Ball(.5,.3);
  r=terminal.Step(in);
  assert(r.active_mission==MissionType::kLine);
  assert(r.command.action==MissionAction::kNone);
  assert(r.command.mission_phase==4);

  // directional recovery는 정확히 설정된 5회만 TURN하고 그 뒤 terminal이다.
  auto turn_config=c;
  turn_config.line_p2p.failure_observation_sec=0.0;
  turn_config.line_p2p.recovery_max_turns=5;
  MissionController turning(turn_config);
  in=Frame(0); in.line_decision_guide_override=Guide(-.3);
  r=turning.Step(in);
  auto action_id=r.command.action_id;
  in=Frame(.1); in.line_centers.clear();
  in.delivery_feedback={action_id,true,true,false};
  r=turning.Step(in);
  in=Frame(.2); in.line_centers.clear(); r=turning.Step(in);
  for (int turn=1; turn<=5; ++turn) {
    assert(r.command.action==MissionAction::kTurnLeft);
    assert(r.command.target_yaw_deg==15);
    action_id=r.command.action_id;
    const double done_time=.2+turn*.2-.1;
    in=Frame(done_time); in.line_centers.clear();
    in.delivery_feedback={action_id,true,true,false};
    r=turning.Step(in);
    in=Frame(done_time+.1); in.line_centers.clear();
    r=turning.Step(in);
  }
  assert(r.command.action==MissionAction::kNone);
  assert(r.command.mission_phase==4);
  return 0;
}
