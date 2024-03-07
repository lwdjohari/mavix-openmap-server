#pragma once

#include <mavix/v1/core/core.h>

#include <algorithm>
#include <fstream>

#include "absl/container/node_hash_map.h"
#include "mavix/v1/core/buffer_struct.h"
#include "mavix/v1/core/memory_buffer.h"
#include "mavix/v1/core/page_locator.h"
#include "mavix/v1/core/stream.h"

namespace mavix {
namespace v1 {
namespace core {

enum class CacheGenerationOptions { None = 0, LimitMaxCacheSize = 1 };


NVM_ENUMCLASS_ENABLE_BITMASK_OPERATORS(CacheGenerationOptions)

NVM_ENUM_CLASS_DISPLAY_TRAIT(CacheGenerationOptions)



/**
 * @brief Class that handle operations of multi-page/multi chunk stream buffered
 */
class CacheBucket {
 private:
  size_t cache_size_page_;
  size_t max_cache_size_;
  std::streamsize stream_size_;
  std::shared_ptr<ICacheBucketBuffer> stream_;
  size_t number_of_max_cache_page_;
  uint64_t last_prepend_cache_page_;
  AFlagOnce isInitialized_;
  absl::node_hash_map<uint64_t, MemoryBuffer> caches_;
  absl::node_hash_map<uint64_t, BufferPage> pages_;
  absl::node_hash_map<uint64_t, BufferPage> active_pages_;
  absl::node_hash_map<uint64_t, BufferPage> deleted_pages_;
  PageLocator buffer_locator_;
  CacheGenerationOptions options_;

 public:
  explicit CacheBucket(std::shared_ptr<ICacheBucketBuffer> stream,
                       CacheGenerationOptions options,
                       const size_t& cache_size_page = 1024 * 1024 * 20,
                       const size_t& max_cache_size = 1024 * 1024 * 20 * 10)
      : stream_(stream),
        options_(options),
        stream_size_(stream->Size()),
        cache_size_page_(size_t(cache_size_page)),
        max_cache_size_(std::streamsize(max_cache_size)),
        number_of_max_cache_page_(0),
        caches_(absl::node_hash_map<uint64_t, MemoryBuffer>()),
        pages_(absl::node_hash_map<uint64_t, BufferPage>()),
        active_pages_(absl::node_hash_map<uint64_t, BufferPage>()),
        deleted_pages_(absl::node_hash_map<uint64_t, BufferPage>()),
        last_prepend_cache_page_(0),
        buffer_locator_(PageLocator()),
        isInitialized_(AFlagOnce()) {
    Initialize();
  }

  ~CacheBucket() {
    // if (isInitialized_.State()) {
    //   Destroy();
    // }
  }

  void Initialize() {
    stream_size_ = stream_->Size();

    if (IsOptionLimitMaxCacheSize()) {
      number_of_max_cache_page_ = GetMaxCachePageNumbers();
    }

    pages_ = GetRequiredBufferPages();

    isInitialized_.Signal();
  }

  bool IsOptionLimitMaxCacheSize() {
    return (options_ & CacheGenerationOptions::LimitMaxCacheSize) ==
           CacheGenerationOptions::LimitMaxCacheSize;
  }

  size_t GetMaxCachePageNumbers() {
    if (stream_size_ == 0) return 0;
    if (stream_size_ < max_cache_size_) return 1;

    return buffer_locator_.GetTotalPages(max_cache_size_, cache_size_page_);
  }

  size_t MaterializeCachePages(std::streampos position, size_t size) {
    auto locators = buffer_locator_.GetPageRange(
        position, size, cache_size_page_, stream_size_);

    if (!locators->state) return 0;

    if (locators->type == PageLocatorResolvement::StartPageResolve) {
      // we could lead to undefined behavior
      // creating buffer larger or smaller than cache_size_page_
      return 0;
    }

    auto page_nums = locators->end_page_id - locators->start_page_id;
    auto materialized_count = 0;
    for (auto i = locators->start_page_id; i <= locators->end_page_id; i++) {
      if (active_pages_.contains(i)) {
        materialized_count++;
        continue;
      }

      auto buff = caches_.emplace(i, MemoryBuffer(size));
      auto page = &pages_.at(i);
      auto copy_state = stream_->CopyToPointer(buff.first->second.Data(),
                                               page->start, page->size);
      page->cache_page_state = BufferPageState::Allocated;
      page->marked_pos = std::streampos(0);
      page->marked_size = std::streamsize(0);

      active_pages_.emplace(i, BufferPage(*page));

      materialized_count++;
    }

    return materialized_count;
  }

  size_t RemoveCaches(const std::streampos& pos, const std::streamsize& size) {
    auto locators =
        buffer_locator_.GetPageRange(pos, size, cache_size_page_, stream_size_);
    if (!locators->state) return 0;

    if (locators->type == PageLocatorResolvement::StartPageResolve) return 0;

    size_t removed = 0;
    for (uint64_t i = locators->start_page_id; i <= locators->end_page_id;
         i++) {
      if (!active_pages_.contains(i)) {
        removed++;
        continue;
      }

      caches_.erase(i);
      active_pages_.erase(i);
      removed++;
    }

    return removed;
  }

  size_t RemoveCaches(uint64_t buffer_page_id) {
    if (!active_pages_.contains(buffer_page_id)) {
      return 0;
    }

    auto c = &caches_.at(buffer_page_id);
    c->Destroy();

    caches_.erase(buffer_page_id);
    active_pages_.erase(buffer_page_id);

#if defined(MAVIX_DEBUG_CORE) && defined(MAVIX_DEBUG_STREAM_BUFFER)
    std::cout << "Removed cache page from bucket [" << buffer_page_id << "]"
              << std::endl;
#endif

    return 1;
  }

  bool Reset() {
    Destroy();
    Initialize();
    return true;
  }

  bool Destroy() {
    for (auto& c : caches_) {
      c.second.Destroy();
    }

    pages_.clear();
    active_pages_.clear();
    deleted_pages_.clear();
    caches_.clear();
    last_prepend_cache_page_ = 0;
    number_of_max_cache_page_ = 0;

    isInitialized_.Reset();
    return true;
  }

  uint8_t* DataInline(uint64_t cache_page_id, std::streampos global_pos,
                      size_t size, bool prepend = false) {
    if (pages_.size() == 0) return nullptr;

    if (!pages_.contains(cache_page_id)) return nullptr;
    auto p = pages_.at(cache_page_id);

    if (!active_pages_.contains(cache_page_id)) {
      if (prepend) {
        auto materialized = MaterializeCachePages(p.start, p.size);
      } else {
        return nullptr;
      }
    }

    auto local_pos = buffer_locator_.TranslateGlobalPosToLocalPos(
        global_pos, stream_size_, cache_size_page_);
    if (local_pos.first == 0) return nullptr;

    auto c = &caches_.at(cache_page_id);
    return c->Data(local_pos.second, size);
  }

  std::vector<BufferPointer> GetAsPointer(std::streampos position, size_t size,
                                          PageLocatorInfo& resolve_result) {
    return std::vector<BufferPointer>();
  }

  std::shared_ptr<MemoryBuffer> GetAsCopy(std::streampos position, size_t size,
                                          PageLocatorInfo& resolve_result) {
    if (pages_.size() == 0) {
      resolve_result = PageLocatorInfo();
      return nullptr;
    }

    auto locator = GetBufferLocator(position, size);

    if (!locator) {
      resolve_result = PageLocatorInfo();
      return nullptr;
    }

    if (locator->type == PageLocatorResolvement::SinglePage) {
      auto buffer = std::make_shared<MemoryBuffer>(size);
      resolve_result.Clone(*locator);
      return std::move(buffer);
    } else if (locator->type == PageLocatorResolvement::CrossPage) {
      auto global_cursor = std::streampos(position);
      auto global_end = position + std::streamsize(size);
      size_t buffer_cursor = 0;

      auto buffer = std::make_shared<MemoryBuffer>(size);

      for (uint64_t i = locator->start_page_id; i <= locator->end_page_id;
           i++) {
        auto isPageDefined = pages_.contains(i);
        if (!isPageDefined) {
          resolve_result = PageLocatorInfo();
          return nullptr;
        }

        BufferPage& p = pages_.at(i);

        // Ensure cache pages are materialized if necessary
        if (!active_pages_.contains(i)) {
          MaterializeCachePages(p.start, p.size);
        }

        // Calculate the start of the copy range within the current page
        auto local_start = std::max(p.start, global_cursor);
        // Calculate the end of the copy range within the current page
        // +1 because 'end' is inclusive
        auto copy_end = std::min(global_end, p.end + std::streamsize(1));

        auto local_size = copy_end - local_start;

        if (local_size <= 0) {
          break;
        }

        auto& c = caches_.at(i);
        // Adjusted for page-relative position
        auto src_ptr = c.Data(local_start - p.start, local_size);

        if (!src_ptr) {
          resolve_result = PageLocatorInfo();
          return nullptr;
        }

        auto dest_ptr = buffer->Data(buffer_cursor, local_size);
        std::memcpy(dest_ptr, src_ptr, local_size);

        buffer_cursor += local_size;
        global_cursor = copy_end;
      }

      resolve_result.Clone(*locator);
      return std::move(buffer);
    }

    resolve_result = PageLocatorInfo();
    return nullptr;
  }

  bool IsBufferIsAllocated(std::streampos position, size_t size) {
    return false;
  }

  absl::node_hash_map<uint64_t, BufferPage>& BuferPages() { return pages_; }

  std::shared_ptr<PageLocatorInfo> GetBufferLocator(std::streampos position,
                                                    std::streamsize size) {
    return buffer_locator_.GetPageRange(position, size, cache_size_page_,
                                        stream_size_);
  }

  absl::node_hash_map<uint64_t, BufferPage> GetRequiredBufferPages() {
    return buffer_locator_.GetRequiredPages(cache_size_page_, stream_size_);
  }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
