// 처리 순서: [물체 분기 1단계] YOLO bbox가 공통 Detection으로 변환된 뒤 호출한다.
// 역할: class/confidence/크기 조건으로 공, 골대, 백보드, 허들의 최적 후보를 고른다.
// 값 전달: Detection 목록을 인자로 받고, 선택된 ObjectTargets를 반환한다.
// 다음 단계: 공/골대/허들 후보는 각각의 controller.cpp로 전달한다.

#include "vision_core/object_target_extractor.hpp"

namespace vision_core {
namespace {

ObjectTarget MakeTarget(const Detection &detection) {
  ObjectTarget target;
  target.class_id = detection.class_id;
  target.confidence = detection.confidence;
  target.box_px = detection.box;
  target.width_px = detection.box.width;
  target.height_px = detection.box.height;
  target.area_px = target.width_px * target.height_px;
  target.center_px = {detection.box.x + 0.5 * target.width_px,
                      detection.box.y + 0.5 * target.height_px};
  target.rectified_center_px = target.center_px;
  return target;
}

std::optional<ObjectTarget> ExtractBest(
    const std::vector<Detection> &detections, int class_id,
    double confidence_threshold, const ObjectTargetConfig &config) {
  std::optional<ObjectTarget> best;
  for (const auto &detection : detections) {
    if (detection.class_id != class_id ||
        detection.confidence < confidence_threshold ||
        detection.box.width < config.min_box_width ||
        detection.box.height < config.min_box_height) {
      continue;
    }
    const ObjectTarget candidate = MakeTarget(detection);
    if (!best || candidate.confidence > best->confidence ||
        (candidate.confidence == best->confidence &&
         candidate.area_px > best->area_px)) {
      best = candidate;
    }
  }
  return best;
}

} // namespace

ObjectTargets ExtractObjectTargets(const std::vector<Detection> &detections,
                                   const ObjectTargetConfig &config) {
  ObjectTargets targets;
  targets.ball = ExtractBest(detections, config.ball_class_id,
                             config.ball_confidence, config);
  targets.goal = ExtractBest(detections, config.goal_class_id,
                             config.goal_confidence, config);
  targets.backboard = ExtractBest(detections, config.backboard_class_id,
                                  config.backboard_confidence, config);
  targets.hurdle = ExtractBest(detections, config.hurdle_class_id,
                               config.hurdle_confidence, config);
  return targets;
}

} // namespace vision_core
