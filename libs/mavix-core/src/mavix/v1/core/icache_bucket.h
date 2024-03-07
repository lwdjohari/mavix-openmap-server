#pragma once
#include <fstream>

namespace mavix {
namespace v1 {
namespace core {
    class ICacheBucketBuffer{
  public:
    virtual std::streamsize Size() const = 0;
    virtual bool CopyToPointer(uint8_t* data, std::streampos start, size_t size)  = 0;
};
}  // namespace core

}  // namespace v1
}  // namespace mavix

