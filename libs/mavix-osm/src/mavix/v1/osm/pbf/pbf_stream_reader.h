#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <functional>
#include <memory>

#include "mavix/v1/core/aflag_once.h"
#include "mavix/v1/core/memory_buffer.h"
#include "mavix/v1/core/stream.h"
#include "mavix/v1/core/stream_buffer.h"
#include "mavix/v1/osm/block_type.h"
#include "mavix/v1/osm/pbf/pbf_decoder.h"
#include "mavix/v1/osm/pbf/pbf_tokenizer.h"
#include "mavix/v1/osm/skip_options.h"
namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

/// @brief  Class for decoding PBF file into OSM Data Structures.
///         This class designed for multi-threaded processing by utilized
///         one-producer and multi-consumer approach.
///         Use wait() method for waiting until decoder finished all processing.
/// @author Linggawasistha Djohari <linggawasistha.djohari@outlook.com>
class PbfStreamReader {
 private:
 protected:
  void (*on_tokenizer_start_callback_)(PbfTokenizer* sender,
                                       core::StreamState state);

  void (*on_tokenizer_err_)(PbfTokenizer* sender, pbf::PbfTokenizerErr err);

  void (*on_tokenizer_finished_callback_)(PbfTokenizer* sender,
                                          core::StreamState state);

  std::function<void(PbfTokenizer*, std::shared_ptr<pbf::PbfBlobData>)>
      on_pbf_raw_blob_ready_;

  void (*on_osm_data_ready_)(PbfTokenizer* sender,
                             std::shared_ptr<osm::ElementBase> osm_element);

  std::string file_;
  size_t cache_size_;
  std::shared_ptr<core::StreamBuffer> stream_;
  SkipOptions options_;
  core::AFlagOnce isRun_;
  bool verbose_;

  virtual void Process() {
    auto tokenizer = pbf::PbfTokenizer(stream_->GetAdapter(), verbose_);

    if (on_tokenizer_err_) tokenizer.OnDataError(on_tokenizer_err_);
    if (on_pbf_raw_blob_ready_) tokenizer.OnDataReady(on_pbf_raw_blob_ready_);
    if (on_tokenizer_start_callback_)
      tokenizer.OnStarted(on_tokenizer_start_callback_);
    if (on_tokenizer_finished_callback_)
      tokenizer.OnFinished(on_tokenizer_finished_callback_);

    tokenizer.Split();

    tokenizer.OnDataErrorUnregister();
    tokenizer.OnDataReadyUnregister();
    tokenizer.OnStartedUnregister();
    tokenizer.OnFinishedUnregister();
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
        stream_(std::make_shared<core::StreamBuffer>(
            file, cache_options, processing_cache_size,
            processing_cache_size * 20)),
        options_(options),
        isRun_(core::AFlagOnce()),
        verbose_(verbose),
        on_osm_data_ready_(nullptr),
        on_pbf_raw_blob_ready_(nullptr),
        on_tokenizer_err_(nullptr),
        on_tokenizer_finished_callback_(nullptr),
        on_tokenizer_start_callback_(nullptr){
            // Initialize(file_, cache_size_);
        };

  explicit PbfStreamReader(const std::string& file, bool verbose = true)
      : file_(std::string(file)),
        cache_size_(1024 * 1024 * 20),
        stream_(std::make_shared<core::StreamBuffer>(
            file, core::CacheGenerationOptions::None, 1024 * 1024 * 20,
            1024 * 1024 * 20 * 20)),
        options_(SkipOptions::None),
        isRun_(core::AFlagOnce()),
        verbose_(verbose),
        on_osm_data_ready_(nullptr),
        on_pbf_raw_blob_ready_(nullptr),
        on_tokenizer_err_(nullptr),
        on_tokenizer_finished_callback_(nullptr),
        on_tokenizer_start_callback_(nullptr){
            // Initialize(file_, cache_size_);
        };

  ~PbfStreamReader() {
    if (stream_->IsOpen()) {
      stream_->Close();
    }

    on_pbf_raw_blob_ready_ = nullptr;
  };

  std::string GetFilename() const { return stream_->GetFilename(); };

  std::string GetDirectoryPath() const { return stream_->GetDirectoryPath(); };

  std::string GetDirectorySeparatorPath() const {
    return stream_->GetDirectorySeparatorPath();
  };

  std::string GetFileExtension() const { return stream_->GetFileExtension(); };

  const std::string& Filename() const { return file_; }

  bool IsStreamOpen() const { return stream_->IsOpen(); }

  bool IsStreamGood() const { return stream_->IsGood(); }

  bool IsStreamEof() const { return stream_->IsEof(); }

  std::streamsize StreamSize() const { return stream_->Size(); }

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

  SkipOptions DecoderOptions() const{
    return options_;
  }

  core::StreamState Stop(bool auto_close = true) {
    if (!isRun_.State()) return core::StreamState::Stoped;

    isRun_.Reset();

    if (auto_close) {
      Close();
    }

    return core::StreamState::Ok;
  }

  // void Initialize(const std::string& file,
  //                 const size_t& processing_cache_size) {}

  void OnDataReady(
      std::function<void(PbfTokenizer*, std::shared_ptr<pbf::PbfBlobData>)>
          callback) {
    on_pbf_raw_blob_ready_ = callback;
  }

  void OnDataReadyUnregister() { on_pbf_raw_blob_ready_ = nullptr; }

  void OnDataError(void (*callback)(PbfTokenizer* sender,
                                    PbfTokenizerErr err)) {
    on_tokenizer_err_ = callback;
  }

  void OnDataErrorUnregister() { on_tokenizer_err_ = nullptr; }

  void OnStarted(void (*callback)(PbfTokenizer* sender, StreamState state)) {
    on_tokenizer_start_callback_ = callback;
  }

  void OnStartedUnregister() { on_tokenizer_start_callback_ = nullptr; }

  void OnFinished(void (*callback)(PbfTokenizer* sender, StreamState state)) {
    on_tokenizer_finished_callback_ = callback;
  }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix