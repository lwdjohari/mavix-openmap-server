#include <iostream>

#include "mavix/v1/core/shared_buffer.h"
using namespace mavix::v1::core;

int main() {
  auto buffer = memory::make_shared_with_allocator<SharedBuffer>(0, 1920*1080);
  std::cout << "AllocatorType: " << buffer->AllocatorType() << std::endl;
  std::cout << "Allocator (IsAllocated = " << buffer->IsAllocated()
            << ", size = " << buffer->Size() << std::endl;
  return 0;
}