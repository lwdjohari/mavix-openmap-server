#pragma once

#include <mavix/v1/core/core.h>
#include <mavix/v1/core/memory_buffer.h>

#include <algorithm>
#include <cstdint>

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


struct BufferData {
  uint8_t* ptr;
  size_t size;
  std::shared_ptr<MemoryBuffer> data;
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

  explicit PageLocatorInfo()
      : start_page_id(0),
        end_page_id(0),
        start(std::streampos(0)),
        end(std::streampos(0)),
        total_size(0),
        type(PageLocatorResolvement::Unknown),
        state(false) {}

  PageLocatorInfo(const PageLocatorInfo& other)
      : start_page_id(other.start_page_id),
        end_page_id(other.end_page_id),
        start(other.start),
        end(other.end),
        total_size(other.total_size),
        type(other.type),
        state(other.state) {}

  void Clone(const PageLocatorInfo& other) {
    start_page_id = other.start_page_id;
    end_page_id = other.end_page_id;
    start = std::streampos(other.start);
    end = std::streampos(other.end);
    total_size = std::streamsize(other.total_size);
    type = other.type;
    state = other.state;
  }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
