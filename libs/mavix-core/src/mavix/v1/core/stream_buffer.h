#pragma once

#include <mavix/v1/core/core.h>

#include <algorithm>
#include <fstream>

#include "absl/container/node_hash_map.h"
#include "mavix/v1/core/aflag_once.h"
#include "mavix/v1/core/cache_bucket.h"
#include "mavix/v1/core/stream.h"

namespace mavix {
namespace v1 {
namespace core {
template <typename T>
class IMemoryBufferAdapter {
 public:
  virtual bool IsStreamOpen() const = 0;

  virtual bool IsStreamGood() const = 0;

  virtual bool IsStreamEof() const = 0;

  virtual std::streamsize StreamSize() const = 0;
  virtual size_t CacheSize() const = 0;

  virtual std::shared_ptr<PageLocatorInfo> TryConsume(std::streampos pos,
                                                      std::streamsize size) = 0;

  virtual uint8_t* GetAsInlinePointer(std::streampos pos, std::streamsize size,
                                      PageLocatorResolvement& resolve_result,
                                      bool prepend = true) = 0;

  virtual std::vector<BufferPointer> GetAsPointer(
      std::streampos pos, std::streamsize size,
      PageLocatorResolvement& resolve_result) = 0;

  virtual std::shared_ptr<MemoryBuffer<T>> GetAsCopy(
      std::streampos pos, std::streamsize size,
      PageLocatorResolvement& resolve_result) = 0;

  virtual absl::node_hash_map<uint64_t, BufferPage>
  GetRequiredBufferPages() = 0;

  virtual bool MarkConsume(std::streampos pos, std::streamsize size) = 0;

  virtual size_t RemoveBufferPage(std::streampos pos, std::streamsize size) = 0;

  virtual size_t RemoveBufferPage(uint64_t buffer_page_id) = 0;
};

template <typename T>
class StreamBuffer : IMemoryBufferAdapter<T> {
 private:
  core::AFlagOnce isRun_;
  std::shared_ptr<core::Stream> stream_;
  size_t cache_size_page_;
  size_t max_cache_size_;
  std::string filename_;
  CacheBucket<T> caches_;

 public:
  explicit StreamBuffer(const std::string& filename,
                        CacheGenerationOptions options,
                        const size_t& cache_size_page = 1024 * 1024 * 20,
                        const size_t& max_cache_size = 1024 * 1024 * 20 * 10)
      : filename_(std::string(filename)),
        stream_(memory::make_shared_with_allocator<Stream>(filename)),
        isRun_(AFlagOnce()),
        cache_size_page_(size_t(cache_size_page)),
        max_cache_size_(size_t(max_cache_size)),
        caches_(CacheBucket<T>(stream_, options, cache_size_page,
                               max_cache_size)){};

  ~StreamBuffer() {
    if (IsStreamOpen()) {
      Close();
    }
  };

  const std::string& Filename() const { return filename_; }

  bool IsStreamOpen() const override { return stream_->IsOpen(); }

  bool IsStreamGood() const override { return stream_->IsGood(); }

  bool IsStreamEof() const override { return stream_->IsEof(); }

  std::streamsize StreamSize() const override { return stream_->Size(); }

  size_t CacheSize() const override { return cache_size_page_; }

  std::shared_ptr<PageLocatorInfo> TryConsume(std::streampos pos,
                                              std::streamsize size) override {
    return caches_.GetBufferLocator(pos, size);
  }

  virtual uint8_t* GetAsInlinePointer(std::streampos pos, std::streamsize size,
                                      PageLocatorResolvement& resolve_result,
                                      bool prepend = true) override {
    auto locator = caches_.GetBufferLocator(pos, size);

    if (!locator) {
      resolve_result = PageLocatorResolvement::Unknown;
      return nullptr;
    }

    if (!locator->state) {
      resolve_result = PageLocatorResolvement::Unknown;
      return nullptr;
    }

    if (locator->end_page_id != locator->start_page_id) {
      resolve_result = PageLocatorResolvement::CrossPage;
      return nullptr;
    };

    resolve_result = PageLocatorResolvement::SinglePage;
    return caches_.DataInline(locator->start_page_id, pos, size, prepend);
  }

  std::vector<BufferPointer> GetAsPointer(std::streampos pos,
                                          std::streamsize size,
                                           PageLocatorResolvement& resolve_result) override {
    return caches_.GetAsPointer(pos, size, resolve_result);
  }

  std::shared_ptr<MemoryBuffer<T>> GetAsCopy(
      std::streampos pos, std::streamsize size,
      PageLocatorResolvement& resolve_result) override {
    return caches_.GetAsCopy(pos, size, resolve_result);
  }

  absl::node_hash_map<uint64_t, BufferPage> GetRequiredBufferPages() override {
    return caches_.GetRequiredBufferPages();
  }

  bool MarkConsume(std::streampos pos, std::streamsize size) override {
    return true;
  }

  size_t RemoveBufferPage(std::streampos pos, std::streamsize size) override {
    return caches_.RemoveCaches(pos, size);
  }

  size_t RemoveBufferPage(uint64_t buffer_page_id) override {
    return caches_.RemoveCaches(buffer_page_id);
  }

  IMemoryBufferAdapter<T>* GetAdapter() {
    return this;
  }

  core::StreamState Open() {
    auto state = stream_->Open();
    caches_.Reset();
    return state;
  }

  core::StreamState Close() {
    caches_.Destroy();
    return stream_->Close();
  }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix