#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <algorithm>
#include <mavix/v1/core/memory_buffer.h>
#include "nvm/macro.h"

namespace mavix {
namespace v1 {
namespace core {
/**
 * Stream reader with buffer backup strategy to avoid I/O bottle neck
 */

struct BufferPointer {
  uint8_t* ptr;
  size_t size;
  bool state;
  bool overlap;
};

template <typename T>
struct BufferData {
  uint8_t* ptr;
  size_t size;
  std::shared_ptr<MemoryBuffer<T>> data;
  bool state;
  bool overlap;
};

enum class BufferPageState { Unallocated = 0, Allocated = 1, Deleted = 2 };

NVM_ENUM_CLASS_DISPLAY_TRAIT(BufferPageState)

struct BufferPage {
  std::streampos start;
  std::streampos end;
  std::streamsize size;
  std::streampos marked_pos;
  std::streamsize marked_size;
  uint64_t cache_page_id;
  BufferPageState cache_page_state;
};

enum class PageLocatorResolvement {
  Unknown = 0,
  SinglePage = 1,
  CrossPage = 2,
  StartPageResolve = 3,
  ErrOutOfBound = 4
};

NVM_ENUM_CLASS_DISPLAY_TRAIT(PageLocatorResolvement);

struct PageLocatorInfo {
  uint64_t start_page_id;
  uint64_t end_page_id;
  std::streampos start;
  std::streampos end;
  std::streamsize total_size;
  PageLocatorResolvement type;
  bool state;
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
