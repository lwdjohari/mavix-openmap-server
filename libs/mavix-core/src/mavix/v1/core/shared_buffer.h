#pragma once

#ifndef MAVIX_ALLOCATOR_JEMALLOC
#define MAVIX_ALLOCATOR_JEMALLOC
#endif

#include <cstdint>
#include <memory>
#include "mavix/v1/core/memory/memory_allocator.h"

namespace mavix {
namespace v1 {
namespace core {
    class SharedBuffer
    {
    private:
        size_t size_;
        memory::MemoryAllocator<uint8_t> buffer_;
        uint8_t* begin_;
        uint8_t* end_;
        uint8_t* data_;
        uint64_t id_;
        bool is_allocated_;

    void Initialize();

    public:
        SharedBuffer(uint64_t id, size_t buffer_size);
        ~SharedBuffer();

    const uint64_t &Id() const;
    
    uint8_t* Ptr(size_t index);
    const uint8_t* CPtr(size_t index);
    
    uint8_t* Data();
    const uint8_t* CData() const;
    
    uint8_t* Begin();
    const uint8_t* CBegin();
    
    uint8_t* End();
    const uint8_t* CEnd() const;
    
    size_t Size() const;

    bool IsAllocated() const;
    void Destroy();

    memory::AllocatorType AllocatorType() const;
    
    };
    
}  // namespace core
}  // namespace v1
}  // namespace mavix
