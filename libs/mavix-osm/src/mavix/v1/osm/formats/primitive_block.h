#pragma once

#include <mavix/v1/core/core.h>

#include <memory>
#include <vector>

#include "osmpbf/osmpbf.h"
#include "mavix/v1/osm/element_type.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace formats {

// message PrimitiveBlock {
//   required StringTable stringtable = 1;
//   repeated PrimitiveGroup primitivegroup = 2;

//   // Granularity, units of nanodegrees, used to store coordinates in this
//   block. optional int32 granularity = 17 [default=100];

//   // Offset value between the output coordinates and the granularity grid in
//   units of nanodegrees. optional int64 lat_offset = 19 [default=0]; optional
//   int64 lon_offset = 20 [default=0];

//   // Granularity of dates, normally represented in units of milliseconds
//   since the 1970 epoch. optional int32 date_granularity = 18 [default=1000];
// }

class PrimitiveGroup;

class PrimitiveBlock {
 private:
  int32_t granularity_;
  int32_t date_granularity_;
  int64_t lat_offset_;
  int64_t lon_offset_;
  std::vector<std::string> string_table_;
  std::vector<PrimitiveGroup> primitive_group_;
  OSMPBF::PrimitiveBlock pb_primitive_block_;

 public:
  PrimitiveBlock();
  explicit PrimitiveBlock(const OSMPBF::PrimitiveBlock &primitive_block)
      : pb_primitive_block_(primitive_block),
        granularity_(0),
        date_granularity_(0),
        lat_offset_(0),
        lon_offset_(0){
          
        };

  ~PrimitiveBlock();
};

}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix
