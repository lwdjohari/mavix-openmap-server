#pragma once

#include <mavix/v1/core/core.h>

#include <algorithm>
#include <cstdint>
#include <fstream>

#include "absl/container/node_hash_map.h"
#include "mavix/v1/core/buffer_struct.h"
#include "nvm/macro.h"

namespace mavix {
namespace v1 {
namespace core {

class PageLocator {
 private:
 public:
  PageLocator(){};
  ~PageLocator(){};

  size_t GetTotalPages(const std::streamsize &streamsize,
                       const std::streamsize size_per_page) {
    if (streamsize == 0 || size_per_page == 0) return 0;

    auto stream_size = std::streamsize(streamsize);
    auto page_size = std::streamsize(size_per_page);

    size_t number_pages = stream_size / page_size;

    // Ensure at least one page
    if (number_pages < 1) {
      number_pages = 1;
    }

    return number_pages;
  }

  std::shared_ptr<PageLocatorInfo> GetPageRange(
      std::streampos pos, std::streamsize size,
      const size_t &cache_size_per_page, const std::streamsize &streamsize) {
    auto locator_result = std::make_shared<PageLocatorInfo>();
    auto cache_size = size_t(cache_size_per_page);
    auto stream_size = std::streamsize(streamsize);
    auto position = std::streampos(pos);
    auto requested_size = std::streamsize(size);

    // Check if the requested position is out of bounds
    // cppcheck doing stupid, position could vary and not always true when
    // compare with stream_size

    if (position < 0 || position >= stream_size ||
        // cppcheck-suppress knownConditionTrueFalse
        (position == stream_size && requested_size > 0)) {
      locator_result->type = PageLocatorResolvement::ErrOutOfBound;
      locator_result->state = false;
      return locator_result;
    }

    // Check if the end of the requested range is out of bounds
    // This also covers the case where size could push the range out of bounds
    if (requested_size > 0 && (position + requested_size) > stream_size) {
      locator_result->type = PageLocatorResolvement::ErrOutOfBound;
      locator_result->state = false;
      return locator_result;
    }

    // Calculate the number of pages in the stream.
    // This considers the case where the last page may not be fully utilized.
    uint64_t total_pages = (stream_size + cache_size - 1) / cache_size;

    // Calculate the start page ID based on the position.
    // Ensure it does not exceed total_pages.
    uint64_t start_page_id =
        std::min(static_cast<uint64_t>(position / cache_size) + 1, total_pages);

    locator_result->start_page_id = start_page_id;
    locator_result->start = position;
    locator_result->total_size = requested_size;

    // if size 0 we assume user only asking the start page
    if (requested_size == 0) {
      locator_result->type = PageLocatorResolvement::StartPageResolve;
      locator_result->state = true;
      return locator_result;
    }

    // Calculate the end position of the range, ensuring it does not exceed the
    // stream size.
    std::streampos end_position = std::min(
        std::streamsize(position + requested_size) - 1, stream_size - 1);

    // Calculate the end page ID based on the end position.
    // Ensure it does not exceed total_pages.
    uint64_t end_page_id = std::min(
        static_cast<uint64_t>(end_position / cache_size) + 1, total_pages);

    locator_result->end_page_id = end_page_id;
    locator_result->end = end_position;
    locator_result->type = (end_page_id == start_page_id)
                               ? PageLocatorResolvement::SinglePage
                               : PageLocatorResolvement::CrossPage;
    locator_result->state = true;
    return locator_result;
  }

  absl::node_hash_map<uint64_t, BufferPage> GetRequiredPages(
      const size_t &cache_size_per_page, const std::streamsize &streamsize) {
    auto pages = absl::node_hash_map<uint64_t, BufferPage>();

    if (streamsize == 0) {
      // cppcheck-suppress returnStdMoveLocal
      return std::move(pages);
    }

    uint64_t id = 0;

    auto cache_size = size_t(cache_size_per_page);
    auto stream_size = std::streamsize(streamsize);

    size_t total_pages = ((stream_size + cache_size) - 1) / cache_size;
    for (size_t p = 0; p < total_pages; p++) {
      BufferPage page;
      id++;

      page.start = p * cache_size;
      page.end =
          std::min(static_cast<std::streamsize>(((p + 1) * cache_size) - 1),
                   static_cast<std::streamsize>(stream_size - 1));
      page.size = (page.end - page.start) + 1;
      page.cache_page_id = id;
      page.cache_page_state = BufferPageState::Unallocated;
      page.marked_pos = std::streampos(0);
      page.marked_size = std::streamsize(0);

      pages.emplace(id, std::move(page));
    }

    // cppcheck-suppress returnStdMoveLocal
    return std::move(pages);
  }

  std::pair<uint64_t, size_t> TranslateGlobalPosToLocalPos(
      std::streampos global_position, std::streamsize stream_size,
      size_t cache_size_per_page) {
    if (global_position < 0 || global_position >= stream_size) {
      return {0, 0};
    }

    // Calculate which page the global position falls into (1-based index).
    uint64_t page_id =
        static_cast<uint64_t>(global_position / cache_size_per_page) + 1;

    size_t total_pages = ((stream_size + cache_size_per_page) - 1) / cache_size_per_page;

    // // Determine if this is the last page.
    // bool is_last_page = (page_id == total_pages);

    // // Calculate the start position of the current page, adjusting for 1-based
    // // page_id.
    // std::streamsize page_start_position = (page_id - 1) * cache_size_per_page;

    
    // Calculate the local address (offset within the page) for the global
    // position.
    size_t local_address =
        static_cast<size_t>(global_position % cache_size_per_page);

    return {page_id, local_address};
  }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
