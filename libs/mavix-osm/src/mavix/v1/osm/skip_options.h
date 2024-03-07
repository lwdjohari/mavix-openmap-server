#pragma once

#include <cstdint>
#include <type_traits>
#include "nvm/macro.h"

namespace mavix {
namespace v1 {
namespace osm {
    
enum class SkipOptions : uint8_t {
  None = 0,
  Ways = 1,
  Nodes = 2,
  Relations = 4
};

NVM_ENUMCLASS_ENABLE_BITMASK_OPERATORS(SkipOptions);

}  // namespace osm
}  // namespace v1
}  // namespace mavix
