#pragma once

#include <mavix/v1/core/core.h>

#include <memory>
#include <thread>

#include "absl/container/flat_hash_set.h"
#include "absl/container/node_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"

namespace mavix {
namespace v1 {
namespace core {
class Timer {
 private:
  
 public:
  Timer(){};
  ~Timer(){};

   void Elapsed((*callback)())
};


}  // namespace core

}  // namespace v1

}  // namespace mavix
