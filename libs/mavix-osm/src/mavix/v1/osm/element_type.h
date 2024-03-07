#pragma once

#include <mavix/v1/core/core.h>

#include <ostream>
#include <type_traits>

#include "nvm/macro.h"

namespace mavix {
namespace v1 {
namespace osm {
enum class ElementType {
  Unknown = 0,
  Node = 1,
  Way = 2,
  Relation = 3,
  FileHeader = 4,
};

// cppcheck-suppress unknownMacro
NVM_ENUMCLASS_ENABLE_BITMASK_OPERATORS(ElementType)

NVM_ENUM_CLASS_DISPLAY_TRAIT(ElementType)

}  // namespace osm

}  // namespace v1

}  // namespace mavix
