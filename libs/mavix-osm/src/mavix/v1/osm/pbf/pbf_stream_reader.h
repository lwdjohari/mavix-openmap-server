#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <memory>

#include "mavix/v1/core/aflag_once.h"
#include "mavix/v1/core/memory_buffer.h"
#include "mavix/v1/core/stream.h"
#include "mavix/v1/core/stream_buffer.h"
#include "mavix/v1/osm/block_type.h"
#include "mavix/v1/osm/pbf/pbf_tokenizer.h"
#include "mavix/v1/osm/skip_options.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

class PbfStreamReader {
 private:
 protected:
  std::string file_;
  size_t cache_size_;
  std::shared_ptr<core::StreamBuffer<BlockType>> stream_;
  SkipOptions options_;
  core::AFlagOnce isRun_;
  bool verbose_;

  virtual void Process() {
    auto detector = pbf::PbfTokenizer(verbose_);

    detector.Detect(stream_->GetAdapter(), 0);
  }

 public:
  explicit PbfStreamReader(const std::string& file,
                           SkipOptions options = SkipOptions::None,
                           core::CacheGenerationOptions cache_options =
                               core::CacheGenerationOptions::None,
                           const size_t& processing_cache_size = 1024 * 1024 *
                                                                 20,
                           bool verbose = true)
      : file_(std::string(file)),
        cache_size_(processing_cache_size),
        stream_(std::make_shared<core::StreamBuffer<BlockType>>(
            file, cache_options, processing_cache_size,
            processing_cache_size * 20)),
        options_(options),
        isRun_(core::AFlagOnce()),
        verbose_(verbose) {
    Initialize(file_, cache_size_);
  };

  explicit PbfStreamReader(const std::string& file, bool verbose = true)
      : file_(std::string(file)),
        cache_size_(1024 * 1024 * 20),
        stream_(std::make_shared<core::StreamBuffer<BlockType>>(
            file, core::CacheGenerationOptions::None, 1024 * 1024 * 20,
            1024 * 1024 * 20 * 20)),
        options_(SkipOptions::None),
        isRun_(core::AFlagOnce()),
        verbose_(verbose) {
    Initialize(file_, cache_size_);
  };

  ~PbfStreamReader() {
    if (stream_->IsStreamOpen()) {
      stream_->Close();
    }
  };

  const std::string& Filename() const { return file_; }

  bool IsStreamOpen() const { return stream_->IsStreamOpen(); }

  bool IsStreamGood() const { return stream_->IsStreamGood(); }

  bool IsStreamEof() const { return stream_->IsStreamEof(); }

  std::streamsize StreamSize() const { return stream_->StreamSize(); }

  core::StreamState Open() { return stream_->Open(); }

  core::StreamState Close() { return stream_->Close(); }

  core::StreamState Start(bool verbose = false) {
    if (isRun_.State()) return core::StreamState::Processing;

    if (!IsStreamOpen()) {
      auto open_state = Open();
      if (open_state != core::StreamState::Ok) {
        return open_state;
      }
    }

    isRun_.Signal();

    Process();

    return core::StreamState::Ok;
  }

  core::StreamState Stop(bool auto_close = true) {
    if (!isRun_.State()) return core::StreamState::Stoped;

    isRun_.Reset();

    if (auto_close) {
      Close();
    }

    return core::StreamState::Ok;
  }

  void Initialize(const std::string& file,
                  const size_t& processing_cache_size) {}
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix