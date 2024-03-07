#include <mavix/v1/core/core.h>

#include <algorithm>
#include <cstdint>
#include <sstream>

#include "absl/container/node_hash_map.h"
#include "mavix/v1/osm/element_base.h"
#include "nvm/option.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace formats {

using namespace nvm;

class Way : public ElementBase {
 private:
  std::vector<int64_t> nodes_;

 public:
  explicit Way(int64_t id,
               absl::node_hash_map<std::string, BasicElementProperty> &&props)
      : ElementBase(
            id, ElementType::Way,
            std::forward<
                absl::node_hash_map<std::string, BasicElementProperty>>(props)),
        nodes_(){};
  ~Way(){};

  bool InitializeNodes(const size_t &size) {
    if (size == 0) return false;

    if (nodes_.size() > 0) nodes_.clear();

    nodes_ = std::vector<int64_t>();
    nodes_.reserve(size);

    return true;
  }

  std::vector<int64_t> &Nodes() { return nodes_; }

  std::string ToString() const override {
    std::stringstream info;
    info << "Way id:" << Id() << " {nodes=" << nodes_.size() << ", tags="
         << "}";

    return std::move(info.str());
  }
};

}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix