#include <iostream>

#include "mavix/v1/core/shared_buffer.h"
#include "mavix/v1/core/stream.h"

using namespace mavix::v1::core;

int main() {
  auto buffer = memory::make_shared_with_allocator<SharedBuffer<uint32_t>>(0, 1920*1080);
  std::cout << "AllocatorType: " << buffer->AllocatorType() << std::endl;
  std::cout << "Allocator (IsAllocated = " << buffer->IsAllocated()
            << ", size = " << buffer->Size() << std::endl;

  auto stream = memory::make_shared_with_allocator<Stream>("./file");

  stream->CopyToSharedBuffer(0,0,10);
  return 0;
}