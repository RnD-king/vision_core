#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "vision_core/types.hpp"

namespace vision_core {

// P2P 라인 의도를 만드는 데 필요한 gain만 둔다. 임계값과 최종 action 구간은
// 기존 P2pMotionConfig가 계속 소유한다.
struct LineP2pConfig {
  double offset_gain{};
  double heading_gain{};
  double curvature_gain{};
};

class LineP2pController {
public:
  LineP2pController(const LineP2pConfig &config,
                    const RuleConfig &line_config);

  MotionCommand Compute(const LineGuide &guide) const;

private:
  LineP2pConfig config_;
  RuleConfig line_config_;
};

// locomotion action 하나가 실행되는 동안 LineGuide를 모은다. DONE 시점에 실제
// 수집 시간의 후반 50%만 남기고, 끝에 가까울수록 1->3 선형 가중치를 준다.
class LineGuideAccumulator {
public:
  void Begin(std::uint64_t action_id, double now_sec);
  void Add(std::uint64_t action_id, double now_sec, const LineGuide &guide);
  std::optional<LineGuide> Finish(std::uint64_t action_id, double now_sec);
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
