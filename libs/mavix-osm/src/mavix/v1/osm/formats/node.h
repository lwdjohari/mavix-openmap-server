#pragma once

#include <mavix/v1/core/core.h>

#include <memory>
#include <sstream>

#include "absl/container/node_hash_map.h"
#include "mavix/v1/osm/element_base.h"
#include "osmpbf/osmpbf.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace formats {
class Node : public ElementBase {
 private:
  double lat_;
  double lon_;

 public:
  Node(int64_t id, double lat, double lon)
      : ElementBase(id, ElementType::Node,
                    absl::node_hash_map<std::string, BasicElementProperty>()),
        lat_(lat),
        lon_(lon){};

  Node(int64_t id, double lat, double lon,
       absl::node_hash_map<std::string, BasicElementProperty> &&props)
      : ElementBase(
            id, ElementType::Node,
            std::forward<
                absl::node_hash_map<std::string, BasicElementProperty>>(props)),
        lat_(lat),
        lon_(lon){};

  ~Node(){};

  double Lat() const { return lat_; }

  double Lon() const { return lon_; }

  std::string ToString() const {
    std::ostringstream info;
    info << "Node: " << Id() << " {lat=" << Lat() << ", lon=" << Lon()
         << ", tags="
         << "}";

    return std::move(info.str());
  }
};

}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix