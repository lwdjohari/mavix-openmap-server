#pragma once

#include <mavix/v1/core/core.h>

#include <memory>

#include "mavix/v1/core/memory_buffer.h"
namespace mavix {
namespace v1 {
namespace utils {

using namespace mavix::v1::core;
class ICompression {
 private:
 public:
  virtual std::shared_ptr<MemoryBuffer> Inflate(
      std::shared_ptr<MemoryBuffer> buffer) = 0;

  virtual std::shared_ptr<MemoryBuffer> Deflate(
      std::shared_ptr<MemoryBuffer> buffer) = 0;

  virtual std::shared_ptr<MemoryBuffer> Deflate(const uint8_t *source,
                                                size_t size, bool &result) = 0;

  virtual std::shared_ptr<MemoryBuffer> Inflate(const uint8_t *source,
                                                size_t size, bool &result) = 0;
};

}  // namespace utils
}  // namespace v1
}  // namespace mavix
