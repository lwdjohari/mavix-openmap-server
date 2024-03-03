#ifndef MAVIX_DEBUG_SHARED_BUFFER
#define MAVIX_DEBUG_SHARED_BUFFER
#endif

#include <mavix/v1/core/core.h>

#include <iostream>

#include "mavix/v1/core/stream.h"
using namespace mavix::v1::core;

int main() {

  auto stream =
      memory::make_unique_with_allocator<Stream>("./osm-example/sample.pbf");

  std::cout << "Pbf file: " << stream->File() << std::endl;
  std::cout << "Open File: " << stream->Open() << std::endl;
  std::cout << "Is file open: " << stream->IsOpen() << std::endl;
  std::cout << "Filesize: " << stream->Size() << std::endl;

  std::shared_ptr<SharedBuffer<int>> buffer;
  if (stream->IsOpen()) {
    buffer = stream->CopyToSharedBuffer(0, 0, stream->Size());
  }

  if (buffer) {
    std::cout << "Buffer Size: " << stream->Size() <<"\n"<< std::endl ;

    for (size_t i = 0; i < buffer->Size(); i++)
    {
        std::cout << *(buffer->Begin() + i) << " " ;
    }
    
    std::cout << "\n";

  }

  stream->Close();

  return 0;
}