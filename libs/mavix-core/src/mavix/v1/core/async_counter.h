#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <type_traits>

#include "absl/synchronization/mutex.h"

namespace mavix {
namespace v1 {
namespace core {
template <typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
class AsyncCounter {
 private:
  T counter_init_;
  T counter_val_;
  T lower_limit_;
  T upper_limit_;
  absl::Mutex mu_;

 public:
  AsyncCounter()
      : counter_init_(),
        counter_val_(),
        lower_limit_(std::numeric_limits<T>::lowest()),
        upper_limit_(std::numeric_limits<T>::max()),
        mu_(){};
  explicit AsyncCounter(T init_value)
      : counter_init_(init_value),
        counter_val_(init_value),
        lower_limit_(std::numeric_limits<T>::lowest()),
        upper_limit_(std::numeric_limits<T>::max()),
        mu_(){};
  ~AsyncCounter(){};

  bool Inc() {
    absl::MutexLock lock(&mu_);

    if (counter_val_ >= upper_limit_) {
      return false;
    }

    counter_val_++;
    return true;
  }

  T Value() {
    absl::MutexLock lock(&mu_);
    return counter_val_;
  }

  bool Dec() {
    absl::MutexLock lock(&mu_);
    if (counter_val_ <= lower_limit_) {
      return false;
    }
    counter_val_--;
    return true;
  }

  void Reset() {
    absl::MutexLock lock(&mu_);
    counter_val_ = counter_init_;
  }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
