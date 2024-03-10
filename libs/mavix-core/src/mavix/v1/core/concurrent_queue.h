#pragma once

#include <mavix/v1/core/core.h>
#include <absl/synchronization/mutex.h>
#include <memory>
#include <queue>

namespace mavix {
namespace v1 {
namespace core {
template <typename T>
class ConcurrentQueue {
 private:
  mutable absl::Mutex mu_;
  std::queue<T> queue_;

 public:
  ConcurrentQueue() = default;
  ~ConcurrentQueue(){};

  bool TryDequeue(T &value) {
    absl::MutexLock lock(&mu_);
    if (queue_.empty()) {
      return false;
    }
    value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  void Enqueue(T &value) {
    absl::MutexLock lock(&mu_);
    queue_.emplace(std::move(value));
  }

  void Clear() {
    absl::MutexLock lock(&mu_);
    std::queue<T> empty;
    std::swap(queue_, empty);
  }

  bool Empty() const {
    absl::MutexLock lock(&mu_);
    return queue_.empty();
  };

  size_t Size() const {
    absl::MutexLock lock(&mu_);
    return queue_.size();
  };
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
