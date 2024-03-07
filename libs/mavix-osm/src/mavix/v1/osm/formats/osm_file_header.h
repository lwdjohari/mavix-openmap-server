#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <sstream>

#include "absl/container/node_hash_map.h"
#include "mavix/v1/osm/element_base.h"
namespace mavix {
namespace v1 {
namespace osm {
namespace formats {

class OSMFileheader : public ElementBase {
 private:
 public:
  explicit OSMFileheader()
      : ElementBase(
            0, ElementType::FileHeader,
            std::move(
                absl::node_hash_map<std::string, BasicElementProperty>())) {}

  ~OSMFileheader() {}

  std::string ToString() const override {
    std::ostringstream info;
    info << "OsmHeader: "
         << "{tags="
         << "}";

    return std::move(info.str());
  };
};

}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix