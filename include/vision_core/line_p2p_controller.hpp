#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "vision_core/cruise_selector.hpp"
#include "vision_core/types.hpp"

namespace vision_core {

// 정상 P2P LINE의 O/H score와 direct 3-action 구간을 정의한다.
struct LineP2pConfig {
  double offset_gain{};
  double heading_gain{};
  double steering_deadband{};
  // 1걸음/제자리회전처럼 짧은 locomotion action이 DONE된 뒤 다음 action을
  // 고르기 전에 정지 상태로 LineGuide를 모으는 시간이다.
  double short_post_collect_sec{};
  // 유효 라인이 없거나 명령이 모든 deadband 안이면 별도 action 없이
  // 현재 자세를 유지하며 새 locomotion 판단을 잠그는 시간이다.
  double no_action_hold_sec{};
};

class LineP2pController {
public:
  explicit LineP2pController(const LineP2pConfig &config);

  CruiseDecision Compute(const LineGuide &guide) const;

private:
  LineP2pConfig config_;
};

// locomotion action 하나가 실행되는 동안 LineGuide를 모은다. DONE 시점에 실제
// 수집 시간의 후반 50%만 남기고, 끝에 가까울수록 1->3 선형 가중치를 준다.
class LineGuideAccumulator {
public:
  void Begin(std::uint64_t action_id, double now_sec);
  void Add(std::uint64_t action_id, double now_sec, const LineGuide &guide);
  std::optional<LineGuide> Finish(std::uint64_t action_id, double now_sec);
  // 짧은 action 뒤의 고정 관측 구간은 전체 유효 표본을 동일 가중 평균한다.
  std::optional<LineGuide> FinishAll(std::uint64_t action_id);
  void Reset();
  bool ActiveFor(std::uint64_t action_id) const;

private:
  struct Sample {
    double now_sec{0.0};
    LineGuide guide;
  };

  std::uint64_t action_id_{0};
  double begin_sec_{0.0};
  std::vector<Sample> samples_;
};

} // namespace vision_core
