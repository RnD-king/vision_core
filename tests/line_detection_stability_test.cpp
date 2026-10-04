#include "vision_core/line_feature_extractor.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
using namespace vision_core;
int main() {
  FeatureConfig c{8,50,5,20,3,10};
  auto f=ComputeLineFeatures({{50,90},{50,70},{50,50}},100,100,c);
  assert(f.guide.valid); assert(std::abs(f.guide.offset)<1e-12);
  assert(std::abs(f.guide.heading_rad)<1e-12);
  assert(!f.guide.curvature_valid);  // curve_min_points 미달
  auto split=ComputeLineFeatures({{50,90},{50,70},{50,50},
                                  {30,20},{40,20},{50,20}},100,100,c);
  assert(split.guide.valid);
  assert(!split.guide.curvature_valid);
  assert(std::isfinite(split.guide.confidence));
  auto short_span=ComputeLineFeatures({{50,90},{50,87},{50,84},
                                      {49,81},{48,78}},100,100,c);
  assert(short_span.guide.valid);
  assert(!short_span.guide.curvature_valid);
  auto curved=ComputeLineFeatures({{50,90},{50,70},{50,50},
                                   {45,30},{35,10}},100,100,c);
  assert(curved.guide.valid);
  assert(curved.guide.curvature_valid);
  auto bad=ComputeLineFeatures({{50,90}},100,100,c);
  assert(!bad.guide.valid);
  return 0;
}
