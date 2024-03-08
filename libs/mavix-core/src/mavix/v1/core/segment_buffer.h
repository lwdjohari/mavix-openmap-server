#pragma once
#include <mavix/v1/core/core.h>

#include <vector>

#include "mavix/v1/core/buffer_struct.h"
#include "mavix/v1/core/memory_buffer.h"

namespace mavix {
namespace v1 {
namespace core {
using namespace mavix::v1::core;
class SegmentBuffer {
 private:
  size_t chunk_size_;
  std::vector<MemoryBuffer> caches_;
  size_t segement_size_;

 public:
  explicit SegmentBuffer(size_t chunk_size)
      : chunk_size_(chunk_size),
        segement_size_(),
        caches_(std::vector<MemoryBuffer>(chunk_size)){};

  ~SegmentBuffer() {}

  bool Add(MemoryBuffer &&buffer) {
    caches_.emplace_back(std::forward<MemoryBuffer>(buffer));
    segement_size_ += buffer.Size();
    return true;
  }

  bool Add(const uint8_t *source, size_t size) {
    if (!source || size == 0 || size > chunk_size_) return false;

    auto buffer = MemoryBuffer(chunk_size_);
    buffer.CopyFrom(source, size);

    return Add(std::move(buffer));
  }

  std::shared_ptr<MemoryBuffer> GetAsMemoryBuffer() {
    if (segement_size_ == 0) return nullptr;

    auto flat_buffer = std::make_shared<MemoryBuffer>(segement_size_);

    size_t position = 0;

    for (size_t i = 0; i < caches_.size(); i++)
    {
      auto cache = &caches_.at(i);
      flat_buffer->CopyFrom(cache->Data()+position,cache->Size());
      position+=cache->Size();
    }
    
    return flat_buffer;
  }

  void Destroy() {
    for (auto &c : caches_) {
      c.Destroy();
    }

    caches_.clear();
  }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
