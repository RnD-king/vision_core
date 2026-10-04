#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

static void Reject(const std::string &yaml, const std::string &from,
                   const std::string &to) {
  std::string bad = yaml;
  const auto pos = bad.find(from); assert(pos != std::string::npos);
  bad.replace(pos, from.size(), to);
  const char *path = "invalid_algorithm.yaml";
  { std::ofstream out(path); out << bad; }
  bool rejected = false;
  try { (void)vision_core::LoadAlgorithmConfig(path); }
  catch (const std::runtime_error &) { rejected = true; }
  std::remove(path); assert(rejected);
}

int main(int argc, char **argv) {
  assert(argc == 2);
  const auto c = vision_core::LoadAlgorithmConfig(argv[1]);
  assert(c.line_p2p.failure_min_valid_samples == 5);
  assert(c.line_p2p.recovery_max_turns == 5);
  assert(c.line_p2p.recovery_turn_yaw_deg == 15);
  assert(c.goal.shoot_yaw_limit_deg == 30.0);
  assert(c.ball.fine_target_u_norm == 0.50);
  std::ifstream in(argv[1]); std::ostringstream text; text << in.rdbuf();
  Reject(text.str(), "offset_gain: 1.0", "offset_gain: .nan");
  Reject(text.str(), "heading_gain: 1.0", "heading_gain: -1.0");
  Reject(text.str(), "steering_deadband: 0.10", "steering_deadband: -0.1");
  Reject(text.str(), "line_stable_min_hits: 7", "line_stable_min_hits: 11");
  Reject(text.str(), "failure_min_valid_samples: 5",
         "failure_min_valid_samples: 0");
  Reject(text.str(), "fine_target_u_norm: 0.50", "fine_target_u_norm: 1.1");
  Reject(text.str(), "smooth_alpha: 0.45", "smooth_alpha: .nan");
  Reject(text.str(), "tilt_down_min_hits: 7", "tilt_down_min_hits: 11");
  Reject(text.str(), "pickup_max_attempts: 3", "pickup_max_attempts: 0");
  Reject(text.str(), "acquire_min_v_norm: 0.60", "acquire_min_v_norm: -0.1");
  Reject(text.str(), "target_u_norm: 0.50", "target_u_norm: .inf");
  Reject(text.str(), "throwing_range_m: 0.40", "throwing_range_m: 0.0");
  Reject(text.str(), "fine_settle_duration_sec: 0.60",
         "fine_settle_duration_sec: -0.1");
  Reject(text.str(), "action_ack_timeout_sec: 10.0",
         "action_ack_timeout_sec: .nan");
  Reject(text.str(), "backboard_min_depth_m: 0.20",
         "backboard_min_depth_m: 6.00");
  return 0;
}
