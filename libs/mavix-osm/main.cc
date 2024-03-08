// #ifndef MAVIX_DEBUG_SHARED_BUFFER
// #define MAVIX_DEBUG_SHARED_BUFFER
// #endif

// #ifndef MAVIX_DEBUG_STREAM_BUFFER
// #define MAVIX_DEBUG_STREAM_BUFFER
// #endif

#ifndef MAVIX_DEBUG_OSM_THREAD
#define MAVIX_DEBUG_OSM_THREAD
#endif

#include <gperftools/malloc_extension.h>
#include <mavix/v1/core/core.h>

#include <filesystem>
#include <iostream>

#include "mavix/v1/core/stream_buffer.h"
#include "mavix/v1/osm/osm_pbf_reader.h"
#include "mavix/v1/osm/pbf/pbf_stream_reader.h"
#include "nvm/strings/readable_bytes.h"

using namespace mavix::v1::core;
using namespace mavix::v1::osm;
using namespace nvm;

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
  std::streamsize processing_size = std::streamsize(0);
  bool print_buffer_page = false;
  bool verbose = false;
  bool loop_test = false;
  bool loop_flag = true;
  auto workers = std::thread::hardware_concurrency();
  // auto workers = 8;

  std::string pbf_files[] = {
                             "/home/txv/osm-data/2024/brunei.osm.pbf",
                             "/home/txv/osm-data/2024/east_timor.osm.pbf",
                             "/home/txv/osm-data/2024/philippines.osm.pbf",
                             "/home/txv/osm-data/2024/thailand.osm.pbf",
                             "/home/txv/osm-data/2024/singapore.osm.pbf",
                             "/home/txv/osm-data/2024/malaysia.osm.pbf",
                             "/home/txv/osm-data/2024/indonesia.osm.pbf",
                             };

  std::cout << "\nMavix Native OSM PBF Library" << std::endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Pbf List" << std::endl;

  for (auto &pbf_file : pbf_files) {
    std::cout << "- " << std::filesystem::path(pbf_file).filename().string()
              << std::endl;
  }
  // std::string line;
  // std::cout << "\nPress any key to star processing... ";
  // std::getline(std::cin, line);

  while (loop_flag) {
    for (auto &pbf_file : pbf_files) {
      auto stream = std::make_unique<pbf::OsmPbfReader>(
          pbf_file, SkipOptions::None, workers , 0,
          verbose);
      // auto stream = std::make_unique<pbf::PbfStreamReader>(pbf_file,
      // verbose);

      // stream->Open();
      stream->Start();
      stream->Join();
      stream->Stop();

      // TCMallocInfo();
    }
    if (!loop_test) {
      loop_flag = false;
    }
  }

  // std::cout << "\nTotal pbf processing size: "
  //           << strings::ConvertBytesToReadableSizeString(processing_size)
  //           << std::endl;
  return 0;
}