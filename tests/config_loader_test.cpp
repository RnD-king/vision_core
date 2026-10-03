#include "vision_core/config_loader.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

void ExpectRejected(const std::string &source, const std::string &from,
                    const std::string &to, const std::string &path) {
  std::string invalid = source;
  const auto position = invalid.find(from);
  assert(position != std::string::npos);
  invalid.replace(position, from.size(), to);
  {
    std::ofstream output(path);
    assert(output.good());
    output << invalid;
  }
  bool rejected = false;
  try {
    (void)vision_core::LoadAlgorithmConfig(path);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  std::remove(path.c_str());
  assert(rejected);
}

} // namespace

int main(int argc, char **argv) {
  assert(argc == 2);
  const auto config = vision_core::LoadAlgorithmConfig(argv[1]);
  assert(config.line_features.max_centers == 8);
  assert(std::abs(config.line_features.guide_fit_rmse_full_scale_px - 20.0) <
         1e-12);
  assert(std::abs(config.line.k_slope - 3.0) < 1e-12);
  assert(std::abs(config.line_p2p.offset_gain - 1.0) < 1e-12);
  assert(std::abs(config.line_p2p.heading_gain - 1.0) < 1e-12);
  assert(std::abs(config.line_p2p.steering_deadband - 0.10) < 1e-12);
  assert(std::abs(config.line_p2p.short_post_collect_sec - 1.0) < 1e-12);
  assert(std::abs(config.line_p2p.no_action_hold_sec - 2.0) < 1e-12);
  assert(std::abs(config.ball.far_speed_scale - 0.90) < 1e-12);
  assert(std::abs(config.ball.approach_u_deadband - 0.025) < 1e-12);
  assert(config.ball.tilt_down_min_hits == 7);
  assert(config.command.locomotion_backend ==
         vision_core::LocomotionBackend::kP2pAction);
  assert(std::abs(config.command.action_ack_timeout_sec - 10.0) < 1e-12);
  assert(std::abs(config.command.p2p.sharp_turn_yaw_threshold - 0.30) <
         1e-12);
  assert(std::abs(config.goal.post_pickup_wait_sec - 3.0) < 1e-12);
  assert(std::abs(config.goal.approach_u_deadband - 0.025) < 1e-12);
  assert(std::abs(config.command.p2p_fine.yaw_deadband - 0.03) < 1e-12);
  assert(std::abs(config.backboard_max_depth_m - 5.0) < 1e-12);
  assert(config.object_association.missing_frame_limit == 5);
  assert(std::abs(config.object_association.max_center_distance_norm - 0.25) <
         1e-12);

  std::ifstream input(argv[1]);
  assert(input.good());
  std::ostringstream buffer;
  buffer << input.rdbuf();
  const std::string yaml = buffer.str();
  const std::string invalid_path = "invalid_vision_algorithm_test.yaml";
  ExpectRejected(yaml, "offset_gain: 1.0", "offset_gain: -1.0",
                 invalid_path);
  ExpectRejected(yaml, "heading_gain: 1.0", "heading_gain: -1.0",
                 invalid_path);
  ExpectRejected(yaml, "steering_deadband: 0.10",
                 "steering_deadband: .nan", invalid_path);
  ExpectRejected(yaml, "far_u_des_norm: 0.50", "far_u_des_norm: 1.50",
                 invalid_path);
  ExpectRejected(yaml, "near_target_u_norm: 0.50",
                 "near_target_u_norm: .inf", invalid_path);
  ExpectRejected(yaml, "search_wz: 0.30\n    target_u_norm: 0.50",
                 "search_wz: 0.30\n    target_u_norm: -0.50", invalid_path);
  ExpectRejected(yaml, "far_u_des_norm: 0.50\n    approach_u_deadband: 0.025",
                 "far_u_des_norm: 0.50\n    approach_u_deadband: -0.01",
                 invalid_path);
  ExpectRejected(yaml, "target_u_norm: 0.50\n    approach_u_deadband: 0.025",
                 "target_u_norm: 0.50\n    approach_u_deadband: -0.01",
                 invalid_path);
  ExpectRejected(yaml, "heading_gain: 1.0", "heading_gain: .nan",
                 invalid_path);
  std::cout << "config loader test passed\n";
  return 0;
}
