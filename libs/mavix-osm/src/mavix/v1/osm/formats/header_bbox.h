#ifndef TRAXOSM_FORMATS_V1_HEADER_BBOX_H
#define TRAXOSM_FORMATS_V1_HEADER_BBOX_H

#include <cstdint>
#include <tuple>

namespace mavix {
namespace v1 {
namespace osm {
namespace formats {

class HeaderBBox {
 private:
  int64_t l_;
  int64_t r_;
  int64_t t_;
  int64_t b_;
  std::string source_;
  std::tuple<int64_t, int64_t, int64_t, int64_t> bbox_;

 public:
  HeaderBBox() : l_(0), r_(0), t_(0), b_(0), bbox_(0, 0, 0, 0), source_() {}

  explicit HeaderBBox(const std::string &source)
      : l_(0),
        r_(0),
        t_(0),
        b_(0),
        bbox_(0, 0, 0, 0),
        source_(std::string(source)) {}

  explicit HeaderBBox(int64_t l, int64_t r, int64_t t, int64_t b,
                      const std::string &source)
      : l_(l),
        r_(r),
        t_(t),
        b_(b),
        bbox_(l, r, t, b),
        source_(std::string(source)) {}

  ~HeaderBBox() {}

  int64_t &Left() { return l_; }

  int64_t &Right() { return r_; }

  int64_t &Top() { return t_; }

  int64_t &Bottom() { return b_; }

  std::tuple<int64_t, int64_t, int64_t, int64_t> &BoundingBox() {
    return bbox_;
  }

  std::tuple<int64_t, int64_t, int64_t, int64_t> CloneBoundingBox() {
    return {int64_t(l_), int64_t(r_), int64_t(t_), int64_t(b_)};
  }
};
}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix
#endif