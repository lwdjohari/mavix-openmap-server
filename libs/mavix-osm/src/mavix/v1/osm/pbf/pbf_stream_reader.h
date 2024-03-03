#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <memory>

#include "mavix/v1/core/aflag_once.h"
#include "mavix/v1/core/shared_buffer.h"
#include "mavix/v1/core/stream.h"
#include "mavix/v1/osm/block_type.h"
#include "mavix/v1/osm/pbf/pbf_block_detector.h"
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
  core::Stream stream_;
  SkipOptions options_;
  std::shared_ptr<core::SharedBuffer<BlockType>> buffer_;
  core::AFlagOnce isRun_;
  bool verbose_;

  virtual void Process() {
    buffer_ =
        stream_.CopyToSharedBuffer(BlockType::RawBinary, 0, stream_.Size());

    auto detector = pbf::PbfBlockDetector(verbose_);

    detector.Detect(buffer_, 0);
  }

 public:
  explicit PbfStreamReader(const std::string& file,
                           SkipOptions options = SkipOptions::None,
                           const size_t& processing_cache_size = 1024 * 1024 *
                                                                 20,
                           bool verbose = true)
      : file_(std::string(file)),
        cache_size_(processing_cache_size),
        stream_(file),
        options_(options),
        isRun_(core::AFlagOnce()),
        verbose_(verbose) {
    Initialize(file_, cache_size_);
  };

  explicit PbfStreamReader(const std::string& file, bool verbose = true)
      : file_(std::string(file)),
        cache_size_(1024 * 1024 * 20),
        stream_(file),
        options_(SkipOptions::None),
        isRun_(core::AFlagOnce()),
        verbose_(verbose) {
    Initialize(file_, cache_size_);
  };

  ~PbfStreamReader() {
    if (stream_.IsOpen()) {
      stream_.Close();
    }
  };

  const std::string& Filename() const { return file_; }

  bool IsStreamOpen() const { return stream_.IsOpen(); }

  bool IsStreamGood() const { return stream_.IsGood(); }

  bool IsStreamEof() const { return stream_.IsEof(); }

  std::streamsize StreamSize() const { return stream_.Size(); }

  core::StreamState Open() { return stream_.Open(); }

  core::StreamState Close() { return stream_.Close(); }

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