#pragma once

#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <new>

#include "nvm/macro.h"

#if defined(MAVIX_ALLOCATOR_GOOGLE_ARENA)
#include <gperftools/tcmalloc.h>
#elif defined(MAVIX_ALLOCATOR_JEMALLOC)

#ifndef JEMALLOC_MANGLE
#define JEMALLOC_MANGLE
#endif

#include <jemalloc/jemalloc.h>

#endif

namespace mavix {
namespace v1 {
namespace core {
namespace memory {

enum class AllocatorType { StdAllocator = 0, GoogleArena = 1, JEMalloc = 2 };

// cppcheck-suppress unknownMacro
NVM_ENUM_CLASS_DISPLAY_TRAIT(AllocatorType)

/**
 * @brief Custom memory allocator with flexible change to use Google Arena
 * (TCMalloc) or jemalloc. Allocator selection macros might be defined
 * externally during compilation. #define MAVIX_ALLOCATOR_GOOGLE_ARENA #define
 * MAVIX_ALLOCATOR_JEMALLOC undefinded both of will fallback to STD::ALLOCATOR
 */
template <typename T>
class MemoryAllocator {
 public:
  using value_type = T;
  using pointer = T*;
  using const_pointer = const T*;
  using size_type = size_t;
  using difference_type = ptrdiff_t;

  template <typename U>
  struct rebind {
    using other = MemoryAllocator<U>;
  };

  MemoryAllocator() noexcept = default;

  template <typename U>
  MemoryAllocator(const MemoryAllocator<U>&) noexcept {}

  pointer allocate(size_type n, const void* hint = 0) {
    pointer p = nullptr;
    size_type bytes_to_allocate = n * sizeof(T);
    if (n > max_size()) {
      handle_error("Attempt to allocate too much memory");
      return nullptr;  // Or throw
    }

#if defined(MAVIX_ALLOCATOR_GOOGLE_ARENA)
    p = static_cast<pointer>(tc_malloc(bytes_to_allocate));
#elif defined(MAVIX_ALLOCATOR_JEMALLOC)
    p = static_cast<pointer>(malloc(bytes_to_allocate));
#else
    p = static_cast<pointer>(std::malloc(bytes_to_allocate));
#endif

    if (p == nullptr) {
      handle_error("Memory allocation failed");
      throw std::bad_alloc(); 
    }

    return p;
  }

  void deallocate(pointer p, size_type n) noexcept {
#if defined(MAVIX_ALLOCATOR_GOOGLE_ARENA)
    tc_free(p);
#elif defined(MAVIX_ALLOCATOR_JEMALLOC)
    free(p);
#else
    std::free(p);
#endif
  }

  size_type max_size() const noexcept {
    return std::numeric_limits<size_type>::max() / sizeof(T);
  }

  template <class U, class... Args>
  void construct(U* p, Args&&... args) {
    new ((void*)p) U(std::forward<Args>(args)...);
  }

  template <class U>
  void destroy(U* p) {
    p->~U();
  }

  AllocatorType AllocatorType() const {
#if defined(MAVIX_ALLOCATOR_GOOGLE_ARENA)
    return AllocatorType::GoogleArena;
#elif defined(MAVIX_ALLOCATOR_JEMALLOC)
    return AllocatorType::JEMalloc;
#else
    return AllocatorType::StdAllocator;
#endif
  }

 private:
  void handle_error(const char* message) const {
    // Error handling strategy here
    // For example, log to stderr or throw with more context
    std::cerr << "MemoryAllocator error: " << message << std::endl;
  }
};

template <typename T1, typename T2>
bool operator==(const MemoryAllocator<T1>&,
                const MemoryAllocator<T2>&) noexcept {
  return true;
}

template <typename T1, typename T2>
bool operator!=(const MemoryAllocator<T1>&,
                const MemoryAllocator<T2>&) noexcept {
  return false;
}

}  // namespace memory
}  // namespace core
}  // namespace v1
}  // namespace mavix
