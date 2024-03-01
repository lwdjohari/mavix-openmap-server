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
  explicit OSMFileheader(bool isSupportedFeatures);
  ~OSMFileheader();
  bool isSupportedFeatures();
};

}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix