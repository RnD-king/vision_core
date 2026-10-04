#include "vision_core/config_loader.hpp"
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
  Reject(text.str(), "fine_target_u_norm: 0.50", "fine_target_u_norm: 1.1");
  Reject(text.str(), "target_u_norm: 0.50", "target_u_norm: .inf");
  return 0;
}
