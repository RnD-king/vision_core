#include "vision_core/line_feature_extractor.hpp"
#include <cassert>
#include <cmath>
using namespace vision_core;
int main() {
  FeatureConfig c{8,50,5,20,3,10};
  auto f=ComputeLineFeatures({{50,90},{50,70},{50,50}},100,100,c);
  assert(f.guide.valid); assert(std::abs(f.guide.offset)<1e-12);
  assert(std::abs(f.guide.heading_rad)<1e-12);
  auto split=ComputeLineFeatures({{50,90},{50,70},{50,50},
                                  {30,20},{40,20},{50,20}},100,100,c);
  assert(split.guide.valid);
  assert(!split.guide.curvature_valid);
  assert(std::isfinite(split.guide.confidence));
  auto bad=ComputeLineFeatures({{50,90}},100,100,c);
  assert(!bad.guide.valid);
  return 0;
}
