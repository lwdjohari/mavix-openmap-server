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
  HeaderBBox() : l_(0), r_(0), t_(0), b_(0), bbox_(0, 0, 0, 0) {}

  HeaderBBox(sint64_t l, sint64_t r, sint64_t t, sint64_t b)
      : l_(l), r_(r), t_(t), b_(b), bbox_(l, r, t, b) {}

  ~HeaderBBox() {}

  sint64_t &left() { return l_; }

  sint64_t &right() { return r_; }

  sint64_t &top() { return t_; }

  sint64_t &bottom() { return b_; }

  std::tuple<sint64_t, sint64_t, sint64_t, sint64_t> &bbox() {
    return bbox_;
  }
};
}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix
#endif