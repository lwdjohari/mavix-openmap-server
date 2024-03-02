#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/time/time.h"
#include "osmpbf/osmpbf.h"
namespace mavix {

namespace v1 {
namespace osm {
namespace pbf {

class OsmFieldDecoder {
 private:
  std::vector<std::string> strings_;
  constexpr static double COORDINATE_SCALING_FACTOR = 0.000000001;
  int32_t dateGranularity_;
  int32_t coordGranularity_;
  uint64_t lonOffset_;
  uint64_t latOffset_;

  int32_t loadStringTable(const OSMPBF::PrimitiveBlock &primitiveBlock) {
    auto st = std::make_unique<OSMPBF::StringTable>(
        OSMPBF::StringTable(primitiveBlock.stringtable()));

    int32_t stringNumbers = st->s_size();
    if (stringNumbers <= 0) return stringNumbers;

    strings_.reserve(stringNumbers);
    for (int32_t i = 0; i < stringNumbers; i++) {
      strings_.emplace_back(std::move(std::string(st->s(i))));
    }

    return stringNumbers;
  }

 public:
  OsmFieldDecoder()
      : coordGranularity_(0),
        latOffset_(0),
        lonOffset_(0),
        dateGranularity_(0),
        strings_(std::vector<std::string>()) {}

  explicit OsmFieldDecoder(const OSMPBF::PrimitiveBlock &primitiveBlock)
      : coordGranularity_(primitiveBlock.granularity()),
        latOffset_(primitiveBlock.lat_offset()),
        lonOffset_(primitiveBlock.lon_offset()),
        dateGranularity_(primitiveBlock.date_granularity()),
        strings_(std::vector<std::string>()) {
    loadStringTable(primitiveBlock);
  }

  ~OsmFieldDecoder() {}

  const double &CoordinateScalingFactor() { return COORDINATE_SCALING_FACTOR; }

  double DecodeLatitude(uint64_t rawLat) {
    return COORDINATE_SCALING_FACTOR *
           (latOffset_ + (coordGranularity_ * rawLat));
  }

  double DecodeLongitude(uint64_t rawLon) {
    return COORDINATE_SCALING_FACTOR *
           (lonOffset_ + (coordGranularity_ * rawLon));
  }

  absl::Time DecodeTimestamp(uint64_t rawTimestamp) {
    // OSM Timestamp is in UnixMilisecond format
    return absl::FromUnixMillis(dateGranularity_ * rawTimestamp);
  }

  const std::vector<std::string> &Strings() { return strings_; }

  std::string &DecodeString(size_t index) { return strings_.at(index); }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix
