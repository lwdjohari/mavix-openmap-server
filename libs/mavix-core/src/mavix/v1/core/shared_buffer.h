#pragma once

#include <cstdint>
#include <memory>

namespace mavix {
namespace v1 {
namespace core {
    class SharedBuffer
    {
    private:
        size_t size_;
        std::allocator <uint8_t> buffer_;
        uint8_t* begin_;
        uint8_t* end_;
        uint8_t* data_;
        uint64_t id_;
        bool is_allocated_;

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
    
    };
    
}  // namespace core
}  // namespace v1
}  // namespace mavix
