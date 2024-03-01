#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "lz4.h"


namespace mavix {
namespace v1 {
namespace utils {
    
class Lz4Compression {
 private:
 public:
  Lz4Compression();
  ~Lz4Compression();
};

}  // namespace utils
}  // namespace v1
}  // namespace mavix
