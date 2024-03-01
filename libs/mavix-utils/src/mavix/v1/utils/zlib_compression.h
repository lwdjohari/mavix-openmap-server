#pragma once

#include <zlib.h>
#include <vector>
#include <cstdint>
#include <iostream>
#include <memory>
#include <cstring>

#include "mavix/v1/core/shared_buffer.h"

namespace mavix {
namespace v1 {
namespace utils {
class ZlibCompression {
 private:
 public:
  ZlibCompression();
  ~ZlibCompression();

  /// @brief Compress stream of bytes and return compressed
  /// @param buffer
  /// @return
  std::shared_ptr<core::SharedBuffer> deflateBytes(
      std::shared_ptr<core::SharedBuffer> buffer);

  void deflateBytes(const uint8_t *source, size_t sourceLength,
                    std::vector<uint8_t> *compressed, bool &result);

  /// @brief
  /// @param buffer
  /// @return
  std::shared_ptr<core::SharedBuffer> inflateBytes(
      std::shared_ptr<core::SharedBuffer> buffer);

  void inflateBytes(const uint8_t *source, size_t sourceLength,
                    std::vector<uint8_t> *decompressed, bool &result);
};

}  // namespace utils
}  // namespace v1
}  // namespace mavix