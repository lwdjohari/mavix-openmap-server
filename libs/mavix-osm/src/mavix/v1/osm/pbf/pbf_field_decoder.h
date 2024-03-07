#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/time/time.h"
#include "nvm/option.h"
#include "osmpbf/osmpbf.h"
namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

using namespace nvm;
class PbfFieldDecoder {
 private:
  std::vector<std::string> string_table_;
  constexpr static double COORDINATE_SCALING_FACTOR = 0.000000001;
  int32_t date_granularity_;
  int32_t coord_granularity_;
  uint64_t lon_offset_;
  uint64_t lat_offset_;

  int32_t LoadStringTable(const OSMPBF::PrimitiveBlock &primitive_block) {
    int32_t string_count = primitive_block.stringtable().s_size();
    if (string_count <= 0) return string_count;

    string_table_.reserve(string_count);
    for (int32_t i = 0; i < string_count; i++) {
      string_table_.emplace_back(
          std::string(primitive_block.stringtable().s(i)));
    }

    return string_count;
  }

 public:
  PbfFieldDecoder()
      : coord_granularity_(0),
        lat_offset_(0),
        lon_offset_(0),
        date_granularity_(0),
        string_table_(std::vector<std::string>()) {}

  explicit PbfFieldDecoder(const OSMPBF::PrimitiveBlock &primitive_block)
      : coord_granularity_(primitive_block.granularity()),
        lat_offset_(primitive_block.lat_offset()),
        lon_offset_(primitive_block.lon_offset()),
        date_granularity_(primitive_block.date_granularity()),
        string_table_(std::vector<std::string>()) {
    LoadStringTable(primitive_block);
  }

  ~PbfFieldDecoder() {}

  const double &CoordinateScalingFactor() const {
    return COORDINATE_SCALING_FACTOR;
  }

  double DecodeLatitude(uint64_t raw_lat) const {
    return COORDINATE_SCALING_FACTOR *
           (lat_offset_ + (coord_granularity_ * raw_lat));
  }

  double DecodeLongitude(uint64_t raw_lon) const {
    return COORDINATE_SCALING_FACTOR *
           (lon_offset_ + (coord_granularity_ * raw_lon));
  }

  absl::Time DecodeTimestamp(uint64_t rawTimestamp) const {
    // OSM Timestamp is in UnixMilisecond format
    return absl::FromUnixMillis(date_granularity_ * rawTimestamp);
  }

  Option<std::string> GetString(const size_t &index) const {
    if (index >= string_table_.size()) return Option<std::string>();

    return Option<std::string>(std::move(std::string(string_table_.at(index))));
  }

  const std::vector<std::string> &StringTable() { return string_table_; }

  Option<std::string> GetFromStringTable(size_t index) const {
    if (index >= string_table_.size()) return Option<std::string>();

    return Option<std::string>(std::string(string_table_.at(index)));
  }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix
