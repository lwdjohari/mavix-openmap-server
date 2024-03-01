#include "mavix/v1/core/shared_buffer.h"

namespace mavix {
namespace v1 {
namespace core {

SharedBuffer::SharedBuffer(uint64_t id, size_t buffer_size)
    : id_(id),
      size_(buffer_size),
      begin_(nullptr),
      end_(nullptr),
      data_(nullptr),
      is_allocated_(false),
      buffer_(memory::MemoryAllocator<uint8_t>()) {
  Initialize();
}

void SharedBuffer::Initialize() {
  if (size_ == 0) throw std::bad_alloc();

  data_ = buffer_.allocate(size_);
  if (!data_) return;

  is_allocated_ = true;
  begin_ = data_;
  end_ = data_ + size_;
}

memory::AllocatorType SharedBuffer::AllocatorType() const {
  return buffer_.AllocatorType();
}

void SharedBuffer::Destroy() {
  if (!is_allocated_) return;

  buffer_.deallocate(data_, size_);
  is_allocated_ = false;
  data_ = nullptr;
  begin_ = nullptr;
  end_ = nullptr;
}

SharedBuffer::~SharedBuffer() {
  if (is_allocated_) {
    Destroy();
  }
}

const uint64_t& SharedBuffer::Id() const { return id_; }

uint8_t* SharedBuffer::Ptr(size_t index) { return data_; }

const uint8_t* SharedBuffer::CPtr(size_t index) { return data_; }

uint8_t* SharedBuffer::Data() { return data_; }

const uint8_t* SharedBuffer::CData() const { return data_; }

uint8_t* SharedBuffer::Begin() { return begin_; }

const uint8_t* SharedBuffer::CBegin() { return begin_; }

uint8_t* SharedBuffer::End() { return end_; }

const uint8_t* SharedBuffer::CEnd() const { return end_; }

size_t SharedBuffer::Size() const { return size_; }

bool SharedBuffer::IsAllocated() const { return is_allocated_; }

}  // namespace core
}  // namespace v1
}  // namespace mavix
