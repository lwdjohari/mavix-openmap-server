#pragma once

#include <mavix/v1/core/core.h>

#include <iostream>
#include <thread>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"

class Semaphore {
 private:
  absl::Mutex mu_;
  absl::CondVar cv_;
  uint16_t def_count_;
  uint16_t count_;

 public:
  explicit Semaphore(uint16_t count = 0) : def_count_(count), count_(count) {}

  void Release() {
    absl::MutexLock lock(&mu_);
    ++count_;
    cv_.Signal();
  }

  uint16_t Count() {
    absl::MutexLock lock(&mu_);
    return count_;
  }

  void Reset() {
    absl::MutexLock lock(&mu_);
    count_ = def_count_;
    cv_.SignalAll();
  }

  void Reset(uint16_t count) {
    absl::MutexLock lock(&mu_);
    def_count_ = count;
    count_ = count;
    cv_.SignalAll();
  }

  void Acquire() {
    absl::MutexLock lock(&mu_);
    while (count_ == 0) {
      cv_.Wait(&mu_);
    }
    --count_;
  }
};