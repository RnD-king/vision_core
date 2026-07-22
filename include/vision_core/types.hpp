#pragma once

namespace vision_core {

struct Point2 {
  double u{0.0};
  double v{0.0};
};

struct Intrinsics {
  double fx{600.0};
  double fy{600.0};
  double cx{320.0};
  double cy{240.0};
};

struct FeatureConfig {
  int max_centers{8};
  double image_center_u{320.0};
  double lookahead_delta_v_px{220.0};
  double lookahead_alpha_normal{0.80};
  double lookahead_alpha_recovery{0.85};
  double recover_enter_nvis{2.0};
  double recover_exit_nvis{3.0};
  double recover_enter_u{0.70};
  double recover_exit_u{0.35};
};

struct Features {
  double u_err_near{0.0};
  double u_err_lookahead{0.0};
  double u_err_ctrl{0.0};
  double slope{0.0};
  double n_visible{0.0};
  double in_recovery{0.0};
  double vx_prev{0.0};
  double wz_prev{0.0};
};

struct RuleConfig {
  int line_stable_window{10};
  int line_stable_min_hits{7};
  double line_reacquire_nvis{3.0};
  double line_reacquire_u{0.35};
  double cmd_vx_min{0.10};
  double cmd_vx_max{1.20};
  double cmd_wz_min{-1.90};
  double cmd_wz_max{1.90};
  double v_base{0.85};
  double k_u{3.00};
  double k_slope{3.40};
  double k_v_u{0.35};
  double k_v_slope{0.35};
  double dv_max{0.12};
  double dw_max{0.40};
  double recover_vx{0.12};
  double recover_wz{0.75};
  double line_recovery_vx_max{0.45};
  double low_visible_n{2.0};
  double no_visible_n{0.5};
  double low_visible_vx{0.18};
  double no_visible_vx{0.10};
  double low_visible_wz_decay{0.90};
  double no_visible_wz_decay{0.95};

  double recover_coast_s{0.30};
  double recover_lookahead_m{0.55};
  double recover_search_delay_s{0.35};
  double recover_sweep_period_s{1.20};
  double recover_search_wz_min{0.22};
  double recover_search_wz_max{0.70};
  double recover_k_bearing{1.35};
  double recover_k_heading{0.45};
  double recover_k_cross_track{0.70};
  double recover_path_backtrack_m{0.35};
  double recover_path_forward_margin_m{1.00};
  double recover_side_memory_alpha{0.18};
};

struct Pose2 {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Command {
  double vx{0.0};
  double wz{0.0};
};

struct Box2 {
  double x{0.0};
  double y{0.0};
  double width{0.0};
  double height{0.0};
};

struct Detection {
  Box2 box;
  double confidence{0.0};
  int class_id{0};
};

struct ObjectTarget {
  int class_id{0};
  double confidence{0.0};
  Box2 box_px;
  Point2 center_px;
  Point2 rectified_center_px;
  double width_px{0.0};
  double height_px{0.0};
  double area_px{0.0};
  bool center_rectified{false};
};

struct MotionCommand {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

} // namespace vision_core
