#ifndef MAVIX_DEBUG_SHARED_BUFFER
#define MAVIX_DEBUG_SHARED_BUFFER
#endif

#ifndef MAVIX_DEBUG_STREAM_BUFFER
#define MAVIX_DEBUG_STREAM_BUFFER
#endif

#include <gperftools/malloc_extension.h>
#include <mavix/v1/core/core.h>

#include <iostream>

#include "mavix/v1/core/stream_buffer.h"
#include "mavix/v1/osm/pbf/pbf_stream_reader.h"
using namespace mavix::v1::core;
using namespace mavix::v1::osm;

void TCMallocInfo() {
  size_t value = 0;

  MallocExtension::instance()->GetNumericProperty(
      "generic.current_allocated_bytes", &value);
  if (value > 0) {
    std::cout << "TCMalloc allocated : " << value << ")." << std::endl;
  } else {
    std::cout << "TCMalloc is not detected." << std::endl;
  }
}

int main() {
  std::string pbf_files[] = {"./osm-example/sample.pbf",
                             "/home/txv/osm-data/2024/singapore.osm.pbf",
                             "/home/txv/osm-data/2024/malaysia.osm.pbf",
                             "/home/txv/osm-data/2024/indonesia.osm.pbf"};

  std::cout << "Mavix Native OSM PBF Reader" << std::endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Decode OSM Pbf." << std::endl;

  for (auto &pbf_file : pbf_files) {
    std::cout << "- " << pbf_file << std::endl;
  }
  std::string line;
  std::cout << "\nPress any key to star processing... ";
  std::getline(std::cin, line);

  // while(true){
    for (auto &pbf_file : pbf_files) {
    auto stream = std::make_unique<pbf::PbfStreamReader>(pbf_file, false);

    auto sb = std::make_unique<StreamBuffer<BlockType>>(
        pbf_file, CacheGenerationOptions::None);

    sb->Open();
    auto cache_pages = sb->GetRequiredBufferPages();
    sb->Close();

    std::cout << "\nPbf file: " << stream->Filename() << std::endl;
    std::cout << "Open File: " << stream->Open() << std::endl;
    std::cout << "Is file open: " << stream->IsStreamOpen() << std::endl;
    std::cout << "Filesize: " << stream->StreamSize() << "\n" << std::endl;

    for (auto &cp : cache_pages) {
      std::cout << "Buffer page: { start = " << cp.second.start
                << ", end = " << cp.second.end << ", size = " << cp.second.size
                << "}" << std::endl;
    }

    std::cout << std::endl;

    stream->Start();

    TCMallocInfo();

    stream->Stop();
  }
  // }
  

  return 0;
}