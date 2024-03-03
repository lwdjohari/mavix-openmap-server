#pragma once

#include <cstdint>
#include <iostream>

#include "nvm/macro.h"

namespace mavix {
namespace v1 {
namespace osm {

enum class BlockType : uint8_t {
  None = 0,
  RawHeader = 1,
  RawBinary = 2,
  PbfCompressedBinary = 4,
  PbfBinary = 5
};

NVM_ENUM_CLASS_DISPLAY_TRAIT(BlockType)

}  // namespace osm
}  // namespace v1
}  // namespace mavix
