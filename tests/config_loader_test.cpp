#include "vision_core/config_loader.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>

int main(int argc, char **argv) {
  assert(argc == 2);
  const auto config = vision_core::LoadAlgorithmConfig(argv[1]);
  assert(config.line_features.max_centers == 8);
  assert(std::abs(config.line.k_slope - 3.0) < 1e-12);
  assert(std::abs(config.ball.far_speed_scale - 0.90) < 1e-12);
  assert(config.ball.tilt_down_min_hits == 7);
  assert(config.command.locomotion_backend ==
         vision_core::LocomotionBackend::kP2pAction);
  assert(std::abs(config.command.p2p_fine.yaw_deadband - 0.03) < 1e-12);
  assert(std::abs(config.backboard_max_depth_m - 5.0) < 1e-12);
  assert(config.object_association.missing_frame_limit == 5);
  assert(std::abs(config.object_association.max_center_distance_norm - 0.25) <
         1e-12);
  std::cout << "config loader test passed\n";
  return 0;
}
