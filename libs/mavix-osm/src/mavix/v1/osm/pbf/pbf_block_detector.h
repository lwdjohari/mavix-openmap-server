#pragma once

#include <mavix/v1/core/core.h>

#include <fstream>
#include <memory>

#include "mavix/v1/core/shared_buffer.h"
#include "mavix/v1/osm/block_type.h"
#include "mavix/v1/osm/pbf/pbf_block_map.h"
#include "nvm/bytes/byte.h"
#include "nvm/option.h"
#include "osmpbf/osmpbf.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

class PbfBlockDetector {
 private:
  bool verbose_;

 public:
  PbfBlockDetector(bool verbose = true) : verbose_(verbose){};
  ~PbfBlockDetector(){};

  int32_t GetHeaderLength(std::shared_ptr<core::SharedBuffer<BlockType>> buffer,
                          size_t& position, bool& state) {
    nvm::bytes::ByteOpResult result;

    uint8_t* ptr = buffer->Data(position, 4);

    auto header_size = nvm::bytes::ToInt32<uint8_t>(
        ptr, 4, result, nvm::bytes::EndianessType::BigEndian);

    position += 4;

    state = true;
    return header_size;
  }

  OSMPBF::Blob GetPbfBlob(std::shared_ptr<core::SharedBuffer<BlockType>> buffer,
                          const size_t& header_size, const size_t& data_size,
                          size_t& position, bool& state) {
    uint8_t* ptr = buffer->Data(position, data_size);

    auto blob = OSMPBF::Blob();
    blob.ParseFromArray(ptr, header_size);

    if (verbose_) {
      std::cout << "Blob { data = " << blob.data_case()
                << ", rawsize = " << blob.raw_size()
                << ", bytesize = " << blob.ByteSizeLong() << " }" << std::endl;
    }

    if (blob.ByteSizeLong() == 0) {
      std::cout << "Blob mismatch :  " << blob.ByteSizeLong() << ":"
                << data_size << std::endl;
      state = false;
      return OSMPBF::Blob();
    }

    position += data_size;
    state = true;
    return std::move(blob);
  }

  OSMPBF::BlobHeader GetPbfHeader(
      std::shared_ptr<core::SharedBuffer<BlockType>> buffer,
      const size_t& header_size, size_t& position, bool& state) {
    uint8_t* ptr = buffer->Data(position, header_size);

    auto header = OSMPBF::BlobHeader();
    header.ParseFromArray(ptr, header_size);

    if (verbose_) {
      std::cout << header.type() << " {indexdata = " << header.indexdata()
                << ", datasize = " << header.datasize()
                << ", bytesize = " << header.ByteSizeLong() << "}" << std::endl;
    }

    if (header.ByteSizeLong() == 0) {
      std::cout << "Header : zero-bytes " << std::endl;
      state = false;
      return OSMPBF::BlobHeader();
    }

    position += header_size;
    state = true;

    return std::move(header);
  }

  nvm::Option<PbfBlockMap> Detect(
      std::shared_ptr<core::SharedBuffer<BlockType>> buffer,
      const size_t& start) {
    if (!buffer || buffer->Size() == 0) return nvm::Option<PbfBlockMap>();

    // Get header size

    size_t position = 0;

    while (position < buffer->Size()) {
      bool state = false;
      auto header_size = GetHeaderLength(buffer, position, state);
      auto header = GetPbfHeader(buffer, header_size, position, state);
      auto blob =
          GetPbfBlob(buffer, header_size, header.datasize(), position, state);
    }

    std::cout << "\nProcess size: " << position << std::endl;

    return nvm::Option<PbfBlockMap>();
  }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix