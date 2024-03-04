#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>

#include "absl/synchronization/mutex.h"
#include "nvm/macro.h"

namespace mavix {
namespace v1 {
namespace core {

/***
 * @brief Atomic Flag Once
 */
class AFlagOnce {
 private:
  bool state_;
  absl::Mutex mu_;

 public:
  AFlagOnce() : state_(false){};
  explicit AFlagOnce(bool cstate) : state_(cstate){};
  ~AFlagOnce(){};

   bool State() {
    absl::ReaderMutexLock lock(&mu_);
    return state_;
  }
  void Signal() {
    absl::ReaderMutexLock lock(&mu_);
    state_ = true;
  }

  void Reset() {
    absl::ReaderMutexLock lock(&mu_);
    state_ = false;
  }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
