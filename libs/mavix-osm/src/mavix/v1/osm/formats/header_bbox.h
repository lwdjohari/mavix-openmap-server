#ifndef TRAXOSM_FORMATS_V1_HEADER_BBOX_H
#define TRAXOSM_FORMATS_V1_HEADER_BBOX_H

#include <cstdint>
#include <tuple>

#include "nvm/types/extended_type.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace formats {

using namespace nvm::types::extended;

class HeaderBBox {
 private:
  sint64_t l_;
  sint64_t r_;
  sint64_t t_;
  sint64_t b_;
  std::tuple<sint64_t, sint64_t, sint64_t, sint64_t> bbox_;

 public:
  HeaderBBox();
  explicit HeaderBBox(sint64_t l, sint64_t r, sint64_t t, sint64_t b);
  ~HeaderBBox();

  sint64_t &left();
  sint64_t &right();
  sint64_t &top();
  sint64_t &bottom();
  std::tuple<sint64_t, sint64_t, sint64_t, sint64_t> &bbox();
};
}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix
#endif