#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/time/time.h"
#include "osmpbf/osmpbf.h"
#include "nvm/option.h"
namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

using namespace nvm;
class PbfFieldDecoder {
 private:
  std::vector<std::string> stringTable_;
  constexpr static double COORDINATE_SCALING_FACTOR = 0.000000001;
  int32_t date_granularity_;
  int32_t coord_granularity_;
  uint64_t lon_offset_;
  uint64_t lat_offset_;

  int32_t LoadStringTable(const OSMPBF::PrimitiveBlock &primitiveBlock) {
    auto st = std::make_unique<OSMPBF::StringTable>(
        OSMPBF::StringTable(primitiveBlock.stringtable()));

    int32_t stringNumbers = st->s_size();
    if (stringNumbers <= 0) return stringNumbers;

    stringTable_.reserve(stringNumbers);
    for (int32_t i = 0; i < stringNumbers; i++) {
      stringTable_.emplace_back(std::move(std::string(st->s(i))));
    }

    return stringNumbers;
  }

 public:
  PbfFieldDecoder()
      : coord_granularity_(0),
        lat_offset_(0),
        lon_offset_(0),
        date_granularity_(0),
        stringTable_(std::vector<std::string>()) {}

  explicit PbfFieldDecoder(const OSMPBF::PrimitiveBlock &primitiveBlock)
      : coord_granularity_(primitiveBlock.granularity()),
        lat_offset_(primitiveBlock.lat_offset()),
        lon_offset_(primitiveBlock.lon_offset()),
        date_granularity_(primitiveBlock.date_granularity()),
        stringTable_(std::vector<std::string>()) {
    LoadStringTable(primitiveBlock);
  }

  ~PbfFieldDecoder() {}

  const double &CoordinateScalingFactor() { return COORDINATE_SCALING_FACTOR; }

  double DecodeLatitude(uint64_t raw_lat) {
    return COORDINATE_SCALING_FACTOR *
           (lat_offset_ + (coord_granularity_ * raw_lat));
  }

  double DecodeLongitude(uint64_t raw_lon) {
    return COORDINATE_SCALING_FACTOR *
           (lon_offset_ + (coord_granularity_ * raw_lon));
  }

  absl::Time DecodeTimestamp(uint64_t rawTimestamp) {
    // OSM Timestamp is in UnixMilisecond format
    return absl::FromUnixMillis(date_granularity_ * rawTimestamp);
  }

  const std::vector<std::string> &StringTable() { return stringTable_; }


  Option<std::string>  GetFromStringTable(size_t index) { 
    if(index > stringTable_.size())
      return Option<std::string>();
    
    return Option<std::string>(std::string(stringTable_.at(index))); }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix
