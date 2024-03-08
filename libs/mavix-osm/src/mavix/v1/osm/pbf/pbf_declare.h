#pragma once

#include <mavix/v1/core/core.h>

#include <sstream>

#include "mavix/v1/core/memory_buffer.h"
#include "osmpbf/osmpbf.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {
using namespace mavix::v1::core;
struct PbfBlobData {
  OSMPBF::BlobHeader header;
  OSMPBF::Blob blob;
  std::shared_ptr<MemoryBuffer> blob_data;

  PbfBlobData() : header(), blob(), blob_data() {}

  PbfBlobData(OSMPBF::BlobHeader header, OSMPBF::Blob blob,
              std::shared_ptr<MemoryBuffer> blob_data)
      : header(header), blob(blob), blob_data(blob_data) {}

  
  std::string ToString() {
    std::stringstream info;
    info << "PbfBlobData {"
         << "header=" << header.type() << ", datasize=" << header.datasize()
         << ", size=" << header.ByteSizeLong()
         << ", blob: " << (blob_data ? blob_data->Size() : 0) << "}"
         << std::endl;

    return std::move(info.str());
  }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix
