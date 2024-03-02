#pragma once

#include <cstdint>
#include <memory>

#include "mavix/v1/core/core.h"

namespace mavix {
namespace v1 {
namespace core {
    
template <typename TId>
class SharedBuffer {
 private:
  size_t size_;
  memory::MemoryAllocator<uint8_t> buffer_;
  uint8_t* begin_;
  uint8_t* end_;
  uint8_t* data_;
  TId id_;
  bool is_allocated_;

  void Initialize() {
    if (size_ == 0) throw std::bad_alloc();

    data_ = buffer_.allocate(size_);
    if (!data_) return;

    is_allocated_ = true;
    begin_ = data_;
    end_ = data_ + size_;
  }

 public:
  SharedBuffer(TId id, size_t buffer_size)
      : id_(id),
        size_(buffer_size),
        begin_(nullptr),
        end_(nullptr),
        data_(nullptr),
        is_allocated_(false),
        buffer_(memory::MemoryAllocator<uint8_t>()) {
    Initialize();
  }

  memory::AllocatorType AllocatorType() const {
    return buffer_.AllocatorType();
  }

  void Destroy() {
    if (!is_allocated_) return;

    buffer_.deallocate(data_, size_);
    is_allocated_ = false;
    data_ = nullptr;
    begin_ = nullptr;
    end_ = nullptr;
  }

  ~SharedBuffer() {
    if (is_allocated_) {
      Destroy();
    }
  }

  const TId& Id() const { return id_; }

  uint8_t* Ptr(size_t index) { return data_; }

  const uint8_t* CPtr(size_t index) { return data_; }

  uint8_t* Data() { return data_; }

  const uint8_t* CData() const { return data_; }

  uint8_t* Begin() { return begin_; }

  const uint8_t* CBegin() { return begin_; }

  uint8_t* End() { return end_; }

  const uint8_t* CEnd() const { return end_; }

  size_t Size() const { return size_; }

  bool IsAllocated() const { return is_allocated_; }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
