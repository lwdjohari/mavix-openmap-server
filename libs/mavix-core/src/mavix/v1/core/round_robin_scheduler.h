#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>

#include "absl/synchronization/mutex.h"
#include "mavix/v1/core/async_counter.h"

namespace mavix {
namespace v1 {
namespace core {
class RoundRobinScheduler {
 private:
  uint16_t worker_number_;
  mutable absl::Mutex mu_;
  uint16_t current_worker_;
  core::AsyncCounter<size_t> dispatched_size_;

 public:
  explicit RoundRobinScheduler(uint16_t worker_number)
      : worker_number_(worker_number), current_worker_(0), dispatched_size_() {}
  ~RoundRobinScheduler() = default;

  uint16_t PreviewNextDispatch() const {
    absl::MutexLock lock(&mu_);
    if (worker_number_ == 0) return 0;
    return (current_worker_ + 1) % worker_number_;  // Zero-indexed
  }

  uint16_t Dispatch() {
    absl::MutexLock lock(&mu_);
    if (worker_number_ == 0) return 0;

    current_worker_ = (current_worker_ + 1) % worker_number_;  // Zero-indexed
    dispatched_size_.Inc();
    return current_worker_;
  }

   size_t DispatchedSize()  {
    absl::MutexLock lock(&mu_);
    return dispatched_size_.Value();
  }

  void Reset(uint16_t worker_number) {
    absl::MutexLock lock(&mu_);
    worker_number_ = worker_number;
    current_worker_ = 0;
    dispatched_size_.Reset();
  }
};
}  // namespace core
}  // namespace v1
}  // namespace mavix
