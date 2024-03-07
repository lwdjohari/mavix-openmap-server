#pragma once

#include <mavix/v1/core/core.h>

#include <algorithm>
#include <fstream>

#include "absl/container/node_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "mavix/v1/core/aflag_once.h"
#include "mavix/v1/core/cache_bucket.h"
#include "mavix/v1/core/stream.h"

namespace mavix {
namespace v1 {
namespace core {

using namespace mavix::v1::core;

class IMemoryBufferAdapter {
 public:
  virtual bool IsOpen() const = 0;

  virtual bool IsGood() const = 0;

  virtual bool IsEof() const = 0;

  virtual std::streamsize Size() const = 0;
  virtual size_t CacheSize() const = 0;

  virtual std::shared_ptr<PageLocatorInfo> TryConsume(std::streampos pos,
                                                      std::streamsize size) = 0;

  virtual uint8_t* GetAsInlinePointer(std::streampos pos, std::streamsize size,
                                      PageLocatorInfo& resolve_result,
                                      bool prepend = true) = 0;

  virtual std::vector<BufferPointer> GetAsPointer(
      std::streampos pos, std::streamsize size,
      PageLocatorInfo& resolve_result) = 0;

  virtual std::shared_ptr<MemoryBuffer> GetAsCopy(
      std::streampos pos, std::streamsize size,
      PageLocatorInfo& resolve_result) = 0;

  virtual absl::node_hash_map<uint64_t, BufferPage>
  GetRequiredBufferPages() = 0;

  virtual bool MarkConsume(std::streampos pos, std::streamsize size) = 0;

  virtual size_t RemoveBufferPage(std::streampos pos, std::streamsize size) = 0;

  virtual size_t RemoveBufferPage(uint64_t buffer_page_id) = 0;
};

class StreamBuffer : public StreamBase, public IMemoryBufferAdapter {
 private:
  AFlagOnce isRun_;
  std::shared_ptr<Stream> stream_;
  size_t cache_size_page_;
  size_t max_cache_size_;
  std::string filename_;
  CacheBucket caches_;
  absl::Mutex mu_;

 public:
  explicit StreamBuffer(const std::string& filename,
                        CacheGenerationOptions options,
                        const size_t& cache_size_page = 1024 * 1024 * 20,
                        const size_t& max_cache_size = 1024 * 1024 * 20 * 10)
      : StreamBase(filename),
        filename_(std::string(filename)),
        stream_(std::make_shared<Stream>(filename)),
        isRun_(AFlagOnce()),
        cache_size_page_(size_t(cache_size_page)),
        max_cache_size_(size_t(max_cache_size)),
        caches_(
            CacheBucket(stream_, options, cache_size_page, max_cache_size)){};

  ~StreamBuffer(){
      // if (IsStreamOpen()) {
      //   Close();
      // }
  };

  const std::string& File() const override { return filename_; }

  bool IsOpen() const override { return stream_->IsOpen(); }

  bool IsGood() const override { return stream_->IsGood(); }

  bool IsEof() const override { return stream_->IsEof(); }

  std::streamsize Size() const override { return stream_->Size(); }

  size_t CacheSize() const override { return cache_size_page_; }

  std::shared_ptr<PageLocatorInfo> TryConsume(std::streampos pos,
                                              std::streamsize size) override {
    return caches_.GetBufferLocator(pos, size);
  }

  virtual uint8_t* GetAsInlinePointer(std::streampos pos, std::streamsize size,
                                      PageLocatorInfo& resolve_result,
                                      bool prepend = true) override {
    auto locator = caches_.GetBufferLocator(pos, size);

    if (!locator) {
      resolve_result = PageLocatorInfo();
      return nullptr;
    }

    if (!locator->state) {
      resolve_result = PageLocatorInfo();
      return nullptr;
    }

    if (locator->end_page_id != locator->start_page_id) {
      resolve_result = PageLocatorInfo();
      resolve_result.type = PageLocatorResolvement::CrossPage;
      return nullptr;
    };

    resolve_result.Clone(*locator);
    absl::ReaderMutexLock lock(&mu_);
    return caches_.DataInline(locator->start_page_id, pos, size, prepend);
  }

  std::vector<BufferPointer> GetAsPointer(
      std::streampos pos, std::streamsize size,
      PageLocatorInfo& resolve_result) override {
    return caches_.GetAsPointer(pos, size, resolve_result);
  }

  std::shared_ptr<MemoryBuffer> GetAsCopy(
      std::streampos pos, std::streamsize size,
      PageLocatorInfo& resolve_result) override {
    absl::ReaderMutexLock lock(&mu_);
    return caches_.GetAsCopy(pos, size, resolve_result);
  }

  absl::node_hash_map<uint64_t, BufferPage> GetRequiredBufferPages() override {
    return caches_.GetRequiredBufferPages();
  }

  bool MarkConsume(std::streampos pos, std::streamsize size) override {
    return true;
  }

  size_t RemoveBufferPage(std::streampos pos, std::streamsize size) override {
    absl::ReaderMutexLock lock(&mu_);
    return caches_.RemoveCaches(pos, size);
  }

  size_t RemoveBufferPage(uint64_t buffer_page_id) override {
    absl::ReaderMutexLock lock(&mu_);
    return caches_.RemoveCaches(buffer_page_id);
  }

  IMemoryBufferAdapter* GetAdapter() { return this; }

  StreamState Open() override {
    absl::ReaderMutexLock lock(&mu_);
    if (isRun_.State()) return StreamState::AlreadyOpen;

    auto state = stream_->Open();
    caches_.Reset();
    return state;
  }

  StreamState Close() override {
    absl::ReaderMutexLock lock(&mu_);
    if (!isRun_.State()) return StreamState::Stoped;

    caches_.Destroy();
    return stream_->Close();
  }

  std::streampos MoveTo(std::streampos pos) override {
    absl::ReaderMutexLock lock(&mu_);
    return std::streampos(-1);
  }

  virtual std::streampos CurrentPosition() override {
    absl::ReaderMutexLock lock(&mu_);
    return std::streampos(-1);
  };

  virtual std::streampos Next(std::streamsize size) override {
    absl::ReaderMutexLock lock(&mu_);
    return std::streampos(-1);
  };

  virtual std::streampos Prev(std::streamsize size) override {
    absl::ReaderMutexLock lock(&mu_);
    return std::streampos(-1);
  };

  virtual std::streampos Next() override { return Next(1); };

  virtual std::streampos Prev() override { return Prev(1); };
};

}  // namespace core
}  // namespace v1
}  // namespace mavix