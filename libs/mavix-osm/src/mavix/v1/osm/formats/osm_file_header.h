#pragma once

#include <cstdint>

namespace mavix {
namespace v1 {
namespace osm {
namespace formats {

class OSMFileheader {
 private:
  bool isSupportedFeatures_;

 public:
  explicit OSMFileheader(bool isSupportedFeature)
      : isSupportedFeatures_(isSupportedFeature) {}

  ~OSMFileheader() {}

  bool isSupportedFeatures() { return isSupportedFeatures_; }
};

}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix