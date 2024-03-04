#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "absl/synchronization/mutex.h"
#include "mavix/v1/core/memory_buffer.h"
#include "nvm/result.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {
enum class ScanState {
  Ok = 0,
  Error = 1,
  ZeroSize = 2,
  NullBuffer = 3,
  ScanRunning = 4,
  NotRunning = 5
};

using PbfStreamScannerStateCallback = void (*)(ScanState state,
                                               std::streampos filesize);
using PbfStreamScannerDataCallback = void (*)(uint64_t seq,
                                              std::streampos index,
                                              size_t length);

class PbfStreamScanner {
 private:
  PbfStreamScannerDataCallback onFoundDataCallback_;
  PbfStreamScannerStateCallback onScanStartCallback_;
  PbfStreamScannerStateCallback onScanFinishedCallback_;

  absl::Mutex mu_;
  std::thread thread_;
  bool isRun_;

  void Scan() {}

 public:
  explicit PbfStreamScanner(std::shared_ptr<std::istream> stream)
      : isRun_(false),
        onFoundDataCallback_(nullptr),
        onScanStartCallback_(nullptr),
        onScanFinishedCallback_(nullptr) {}

  ~PbfStreamScanner() {}

  ScanState Start() {
    if (isRun_) return ScanState::ScanRunning;

    isRun_ = true;
    thread_ = std::thread(&Scan, this);
    return ScanState::Ok;
  }

  void TryJoin() {
    if (!isRun_) return;

    if (thread_.joinable()) {
      thread_.join();
    }
  }

  ScanState Stop() {
    if (!isRun_) return ScanState::NotRunning;

    absl::MutexLock lock(&mu_);
    isRun_ = false;

    return ScanState::Ok;
  }

  void OnFoundDataCallback(PbfStreamScannerDataCallback callback) {
    absl::MutexLock lock(&mu_);
    onFoundDataCallback_ = callback;
  }

  void OnScanStartedCallback(PbfStreamScannerStateCallback callback) {
    absl::MutexLock lock(&mu_);
    onScanStartCallback_ = callback;
  }

  void OnScanFinishedCallback(PbfStreamScannerStateCallback callback) {
    absl::MutexLock lock(&mu_);
    onScanFinishedCallback_ = callback;
  }

  void UnregisterOnFoundDataCallback() {
    absl::MutexLock lock(&mu_);
    onScanFinishedCallback_ = nullptr;
  }

  void UnregisterOnScanStartedCallback() {
    absl::MutexLock lock(&mu_);
    onScanStartCallback_ = nullptr;
  }

  void UnregisterOnScanFinishedCallback() {
    absl::MutexLock lock(&mu_);
    onScanFinishedCallback_ = nullptr;
  }
};
}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix