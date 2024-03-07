#include <iostream>

#include "mavix/v1/core/memory_buffer.h"
#include "mavix/v1/core/stream.h"

using namespace mavix::v1::core;

int main() {
  auto buffer = std::make_shared<MemoryBuffer>( 1920*1080);
  std::cout << "AllocatorType: " << buffer->AllocatorType() << std::endl;
  std::cout << "Allocator (IsAllocated = " << buffer->IsAllocated()
            << ", size = " << buffer->Size() << std::endl;

  auto stream = std::make_shared<Stream>("./file");

  stream->CopyToSharedBuffer(0,10);
  return 0;
}