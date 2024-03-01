#include "mavix/v1/osm/formats/header_bbox.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace formats {

using namespace nvm::types::extended;

HeaderBBox::HeaderBBox() : l_(0), r_(0), t_(0), b_(0), bbox_(0, 0, 0, 0) {}

HeaderBBox::HeaderBBox(sint64_t l, sint64_t r, sint64_t t, sint64_t b)
    : l_(l), r_(r), t_(t), b_(b), bbox_(l, r, t, b) {}

HeaderBBox::~HeaderBBox() {}

sint64_t &HeaderBBox::left() { return l_; }

sint64_t &HeaderBBox::right() { return r_; }

sint64_t &HeaderBBox::top() { return t_; }

sint64_t &HeaderBBox::bottom() { return b_; }

std::tuple<sint64_t, sint64_t, sint64_t, sint64_t> &HeaderBBox::bbox() {
  return bbox_;
}

}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix