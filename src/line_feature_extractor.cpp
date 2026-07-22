// 처리 순서: [점선 분기 2단계] coordinate_rectifier 뒤에 호출한다.
// 역할: 보정된 점선 중심점들로부터 오차, 기울기, 가시 점 개수 등 8개 특징을 만든다.
// 값 전달: 점 목록과 이전 명령/복구 상태를 인자로 받고 Features 구조체를 반환한다.
// 다음 단계: 반환값은 line_velocity_controller.cpp의 Observe()/Compute()로 간다.

#include "vision_core/line_feature_extractor.hpp"

#include <algorithm>
#include <cmath>

namespace vision_core {
namespace {
double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}
} // namespace

Features ComputeLineFeatures(const std::vector<Point2> &input, int image_width,
                             int image_height, bool previous_in_recovery,
                             double vx_prev, double wz_prev,
                             const FeatureConfig &cfg) {
  Features f{};
  f.vx_prev = vx_prev;
  f.wz_prev = wz_prev;
  if (image_width <= 1 || image_height <= 1) {
    f.in_recovery = previous_in_recovery ? 1.0 : 0.0;
    return f;
  }

  std::vector<Point2> points;
  points.reserve(input.size());
  for (const Point2 &point : input) {
    if (std::isfinite(point.u) && std::isfinite(point.v)) {
      points.push_back({Clamp(point.u, 0.0, image_width - 1.0),
                        Clamp(point.v, 0.0, image_height - 1.0)});
    }
  }
  std::sort(points.begin(), points.end(), [](const Point2 &a, const Point2 &b) {
    return (a.v == b.v) ? (a.u < b.u) : (a.v > b.v);
  });
  points.resize(std::min(static_cast<std::size_t>(std::max(1, cfg.max_centers)),
                         points.size()));

  f.n_visible = static_cast<double>(points.size());
  if (points.empty()) {
    f.in_recovery = 1.0;
    return f;
  }

  const double cx = cfg.image_center_u >= 0.0
                        ? cfg.image_center_u
                        : 0.5 * static_cast<double>(image_width);
  const double denom = std::max(cx, 1.0);
  f.u_err_near = (points.front().u - cx) / denom;
  f.u_err_lookahead = f.u_err_near;

  if (points.size() >= 2) {
    double min_v = points.front().v;
    double max_v = points.front().v;
    double sum_v = 0.0;
    double sum_u = 0.0;
    for (const Point2 &point : points) {
      min_v = std::min(min_v, point.v);
      max_v = std::max(max_v, point.v);
      sum_v += point.v;
      sum_u += point.u;
    }
    if (max_v - min_v > 1e-6) {
      const double inv_n = 1.0 / static_cast<double>(points.size());
      const double mean_v = sum_v * inv_n;
      const double mean_u = sum_u * inv_n;
      double var_v = 0.0;
      double cov_vu = 0.0;
      for (const Point2 &point : points) {
        var_v += (point.v - mean_v) * (point.v - mean_v);
        cov_vu += (point.v - mean_v) * (point.u - mean_u);
      }
      if (std::abs(var_v) > 1e-9) {
        const double a = cov_vu / var_v;
        const double b = mean_u - a * mean_v;
        f.slope = a / 120.0;
        const double v_lookahead = Clamp(
            points.front().v - cfg.lookahead_delta_v_px, min_v, max_v);
        f.u_err_lookahead = (a * v_lookahead + b - cx) / denom;
      }
    }
  }

  bool recovery = previous_in_recovery;
  if (f.n_visible <= cfg.recover_enter_nvis ||
      std::abs(f.u_err_near) > cfg.recover_enter_u) {
    recovery = true;
  }
  if (f.n_visible >= cfg.recover_exit_nvis &&
      std::abs(f.u_err_near) < cfg.recover_exit_u) {
    recovery = false;
  }
  f.in_recovery = recovery ? 1.0 : 0.0;
  if (f.n_visible >= 2.0) {
    const double alpha = Clamp(recovery ? cfg.lookahead_alpha_recovery
                                        : cfg.lookahead_alpha_normal,
                               0.0, 1.0);
    f.u_err_ctrl = (1.0 - alpha) * f.u_err_near +
                   alpha * f.u_err_lookahead;
  } else {
    f.u_err_ctrl = f.u_err_near;
  }
  return f;
}

} // namespace vision_core
