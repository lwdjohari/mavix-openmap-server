#include "mavix/v1/core/shared_buffer.h"

namespace mavix {
namespace v1 {
namespace core {

SharedBuffer::SharedBuffer(uint64_t id, size_t buffer_size)
    : id_(id),
      size_(size_),
      begin_(nullptr),
      end_(nullptr),
      data_(nullptr),
      is_allocated_(false),
      buffer_(std::allocator<uint8_t>()) {
  if (buffer_size == 0) throw std::bad_alloc();

  data_ = buffer_.allocate(buffer_size);
  if (!data_) return;

  is_allocated_ = true;
  begin_ = data_;
  end_ = data_ + buffer_size;
}

void SharedBuffer::Destroy() {
    if(!is_allocated_) return;

    buffer_.deallocate(data_,size_);
    is_allocated_ = false;
    data_ = nullptr;
    begin_ = nullptr;
    end_ = nullptr;
}

SharedBuffer::~SharedBuffer() {
    if(is_allocated_){
        Destroy();
    }
}

const uint64_t& SharedBuffer::Id() const { return id_; }

uint8_t* SharedBuffer::Ptr(size_t index) { return nullptr; }

const uint8_t* SharedBuffer::CPtr(size_t index) { return nullptr; }
uint8_t* SharedBuffer::Data() { return nullptr; }
const uint8_t* SharedBuffer::CData() const { return nullptr; }
uint8_t* SharedBuffer::Begin() { return nullptr; }
const uint8_t* SharedBuffer::CBegin() { return nullptr; }
uint8_t* SharedBuffer::End() { return nullptr; }
const uint8_t* SharedBuffer::CEnd() const { return nullptr; }
size_t SharedBuffer::Size() const { return size_t(); }
bool SharedBuffer::IsAllocated() const { return false; }

}  // namespace core
}  // namespace v1
}  // namespace mavix
