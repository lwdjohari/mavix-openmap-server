#pragma once

#include <mavix/v1/core/core.h>

#include <fstream>
#include <memory>

#include "mavix/v1/core/stream_buffer.h"
#include "mavix/v1/osm/block_type.h"
#include "mavix/v1/osm/pbf/pbf_block_map.h"
#include "nvm/bytes/byte.h"
#include "nvm/option.h"
#include "osmpbf/osmpbf.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

class PbfTokenizer {
 private:
  bool verbose_;

 public:
  PbfTokenizer(bool verbose = true) : verbose_(verbose){};
  ~PbfTokenizer(){};

  int32_t GetHeaderLength(core::IMemoryBufferAdapter<BlockType>* buffer,
                          std::streampos& position, bool& state) {
    nvm::bytes::ByteOpResult result;

    auto resolve_result = core::PageLocatorResolvement::Unknown;

    uint8_t* ptr = buffer->GetAsInlinePointer(position, 4, resolve_result);
    int32_t header_size = 0;
    if (resolve_result == core::PageLocatorResolvement::SinglePage) {
      header_size = nvm::bytes::ToInt32<uint8_t>(
          ptr, 4, result, nvm::bytes::EndianessType::BigEndian);
    } else if (resolve_result == core::PageLocatorResolvement::CrossPage) {
      auto raw = buffer->GetAsCopy(position, 4, resolve_result);
      header_size = nvm::bytes::ToInt32<uint8_t>(
          raw->Data(), 4, result, nvm::bytes::EndianessType::BigEndian);
    }

    position += std::streamsize(4);
    state = true;
    return header_size;
  }

  OSMPBF::Blob GetPbfBlob(core::IMemoryBufferAdapter<BlockType>* buffer,
                          const size_t& header_size, const size_t& data_size,
                          std::streampos& position, bool& state) {
    auto resolve_result = core::PageLocatorResolvement::Unknown;
    uint8_t* ptr =
        buffer->GetAsInlinePointer(position, header_size, resolve_result);

    auto blob = OSMPBF::Blob();

    if (resolve_result == core::PageLocatorResolvement::SinglePage) {
      blob.ParseFromArray(ptr, header_size);
    } else if (resolve_result == core::PageLocatorResolvement::CrossPage) {
      auto raw = buffer->GetAsCopy(position, header_size, resolve_result);
      blob.ParseFromArray(raw->Data(), header_size);
    }

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

    // position += static_cast<std::streamsize>(data_size);
    position += std::streamsize(data_size);
    state = true;
    return std::move(blob);
  }

  OSMPBF::BlobHeader GetPbfHeader(core::IMemoryBufferAdapter<BlockType>* buffer,
                                  const size_t& header_size,
                                  std::streampos& position, bool& state) {
    auto resolve_result = core::PageLocatorResolvement::Unknown;
    uint8_t* ptr =
        buffer->GetAsInlinePointer(position, header_size, resolve_result);

    auto header = OSMPBF::BlobHeader();

    if (resolve_result == core::PageLocatorResolvement::SinglePage) {
      header.ParseFromArray(ptr, header_size);
    } else if (resolve_result == core::PageLocatorResolvement::CrossPage) {
      auto raw = buffer->GetAsCopy(position, header_size, resolve_result);
      header.ParseFromArray(raw->Data(), header_size);
    }

    // header.ParseFromArray(ptr, header_size);

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

    position += std::streamsize(header_size);
    state = true;

    return std::move(header);
  }

  nvm::Option<PbfBlockMap> Detect(core::IMemoryBufferAdapter<BlockType>* buffer,
                                  const size_t& start) {
    if (!buffer || buffer->StreamSize() == 0) return nvm::Option<PbfBlockMap>();

    // Get header size

    std::streampos position = 0;

    while (position < buffer->StreamSize()) {
      bool state = false;

      auto header_size = GetHeaderLength(buffer, position, state);
      // std::cout << "Header Length: " << position << std::endl;

      auto header = GetPbfHeader(buffer, header_size, position, state);
      // std::cout << "Header: " << position << std::endl;

      // #if defined(MAVIX_DEBUG_CORE)

      // if(position == 20798786){

      // }
      // #endif

      auto blob =
          GetPbfBlob(buffer, header_size, header.datasize(), position, state);

      // std::cout << "Blob: " << position << std::endl;
    }

    std::cout << "\nProcess size: " << position << std::endl;

    return nvm::Option<PbfBlockMap>();
  }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix