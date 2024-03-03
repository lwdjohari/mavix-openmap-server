#pragma once

#include <mavix/v1/core/core.h>

#include <algorithm>
#include <fstream>

#include "absl/container/node_hash_map.h"
#include "mavix/v1/core/aflag_once.h"
#include "mavix/v1/core/shared_buffer.h"
#include "mavix/v1/core/stream.h"
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
  std::shared_ptr<SharedBuffer<T>> data;
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

template <typename T>
class CacheBucket {
 private:
  size_t cache_size_page_;
  size_t max_cache_size_;
  std::streamsize stream_size_;
  std::shared_ptr<core::Stream> stream_;
  size_t number_of_max_cache_page_;
  bool isAllocated_;
  absl::node_hash_map<uint64_t, SharedBuffer<T>> caches_;
  absl::node_hash_map<uint64_t, BufferPage> pages_;
  absl::node_hash_map<uint64_t, BufferPage> deleted_pages_;

 public:
  explicit CacheBucket(std::shared_ptr<core::Stream> stream,
                       const size_t& cache_size_page = 1024 * 1024 * 20,
                       const size_t& max_cache_size = 1024 * 1024 * 20 * 10)
      : stream_(stream),
        stream_size_(stream->Size()),
        cache_size_page_(size_t(cache_size_page)),
        max_cache_size_(std::streamsize(max_cache_size)),
        number_of_max_cache_page_(0),
        isAllocated_(false),
        caches_(absl::node_hash_map<uint64_t, SharedBuffer<T>>()),
        pages_(absl::node_hash_map<uint64_t, BufferPage>()),
        deleted_pages_(absl::node_hash_map<uint64_t, BufferPage>()) {}

  ~CacheBucket() {
    if (isAllocated_) {
      Destroy();
    }
  }

  size_t GetMaxCachePageNumbers() {
    if (stream_size_ == 0) return 0;
    if (stream_size_ < max_cache_size_) return 1;

    size_t number_of_max_cache_pages = max_cache_size_ / cache_size_page_;

    // Ensure at least one page
    if (number_of_max_cache_pages < 1) {
      number_of_max_cache_pages = 1;
    }

    return number_of_max_cache_pages;
  }

  bool BuildCache() {
    pages_ = GetRequiredBufferPages();
    number_of_max_cache_page_ = GetMaxCachePageNumbers();

    for (uint64_t i = 0; i < number_of_max_cache_page_; i++) {
      auto buff = caches_.emplace(i, SharedBuffer<T>(cache_size_page_));
      auto page = &pages_.at(i);
      stream_->CopyToPointer(buff->second.Data(), page->start, page->size);
      page->cache_page_state = BufferPageState::Allocated;
      page->marked_pos = std::streampos(0);
      page->marked_size = std::streamsize(0);
    }

    return true;
  }

  bool Destroy() {
    if (isAllocated_) return false;

    pages_.clear();

    return true;
  }

  absl::node_hash_map<uint64_t, BufferPage>& BuferPages() { return pages_; }

  absl::node_hash_map<uint64_t, BufferPage> GetRequiredBufferPages() {
    auto pages = absl::node_hash_map<uint64_t, BufferPage>();

    if (!stream_->IsOpen()) {
      // cppcheck-suppress returnStdMoveLocal
      return std::move(pages);
    }

    uint64_t id = 0;

    auto cache_size = size_t(cache_size_page_);
    auto stream_size = std::streamsize(stream_size_);

    std::streamsize total_pages = (stream_size + cache_size - 1) / cache_size;
    for (std::streamsize p = 0; p < total_pages; p++) {
      BufferPage page;
      page.start = p * cache_size;
      page.end =
          std::min(static_cast<std::streamsize>((p + 1) * cache_size - 1),
                   static_cast<std::streamsize>(stream_size - 1));
      page.size = page.end - page.start + 1;
      page.cache_page_id = id;
      page.cache_page_state = BufferPageState::Unallocated;
      page.marked_pos = std::streampos(0);
      page.marked_size = std::streamsize(0);

      pages.emplace(id, std::move(page));
      id++;
    }

    // cppcheck-suppress returnStdMoveLocal
    return std::move(pages);
  }
};

template <typename T>
class StreamBuffer {
 private:
  core::AFlagOnce isRun_;
  std::shared_ptr<core::Stream> stream_;
  size_t cache_size_page_;
  size_t max_cache_size_;
  std::string filename_;
  CacheBucket<T> caches_;
  // absl::flat_hash_set<SharedBuffer<T>> buffers_;

  void CreateBuffer(std::streampos pos) {
    auto pages = StreamSize() % cache_size_page_;
  }

 public:
  explicit StreamBuffer(const std::string& filename,
                        const size_t& cache_size_page = 1024 * 1024 * 20,
                        const size_t& max_cache_size = 1024 * 1024 * 20 * 10)
      : filename_(std::string(filename)),
        stream_(memory::make_shared_with_allocator<Stream>(filename)),
        isRun_(AFlagOnce()),
        cache_size_page_(size_t(cache_size_page)),
        max_cache_size_(size_t(max_cache_size)),
        caches_(CacheBucket<T>(stream_, cache_size_page, max_cache_size)){};

  ~StreamBuffer() {
    if (IsStreamOpen()) {
      Close();
    }
  };

  const std::string& Filename() const { return filename_; }

  bool IsStreamOpen() const { return stream_->IsOpen(); }

  bool IsStreamGood() const { return stream_->IsGood(); }

  bool IsStreamEof() const { return stream_->IsEof(); }

  std::streamsize StreamSize() const { return stream_->Size(); }

  size_t CacheSize() const { return cache_size_page_; }

  BufferPointer TryConsume(std::streampos pos, std::streamsize size) {
    return BufferPointer();
  }

  BufferPointer GetAsPointer(std::streampos pos, std::streamsize size) {
    return BufferPointer();
  }

  BufferData<T> GetAsCopy(std::streampos pos, std::streamsize size) {
    return BufferData<T>();
  }

  absl::node_hash_map<uint64_t, BufferPage> GetRequiredBufferPages() {
    return caches_.GetRequiredBufferPages();
  }

  bool MarkConsume(std::streampos pos, std::streamsize size) { return true; }

  bool MarkConsumeToEndAndDropBuffer(std::streampos pos, std::streamsize size) {
    return true;
  }

  core::StreamState Open() {
    auto state = stream_->Open();
    caches_ = CacheBucket<T>(stream_, cache_size_page_, max_cache_size_);
    return state;
  }

  core::StreamState Close() { return stream_->Close(); }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix