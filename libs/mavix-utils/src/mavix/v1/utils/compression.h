#pragma once

#include "mavix/v1/utils/icompression.h"
#include "mavix/v1/utils/zlib_compression.h"

namespace mavix {
namespace v1 {
namespace utils {
template <typename T>
T GetCompression() {
  return T();
}
}  // namespace utils
}  // namespace v1
}  // namespace mavix
