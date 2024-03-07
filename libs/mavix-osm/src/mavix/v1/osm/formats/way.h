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
  std::vector<std::string> nodes_;

 public:
  explicit Way(int64_t id)
      : ElementBase(id, ElementType::Way,
                    absl::node_hash_map<std::string, BasicElementProperty>()),
        nodes_(){};
  ~Way(){};

  std::vector<std::string> Nodes() { return nodes_; }

  std::string ToString() const override {
    std::stringstream info;
    info << "Way id:" + Id() << " {nodes=" << nodes_.size() + ", tags="
         << "}";

    return std::move(info.str());
  }
};

}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix