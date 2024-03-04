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
                          std::streampos& position, core::PageLocatorInfo& resolve_result) {
    nvm::bytes::ByteOpResult result;

    

    uint8_t* ptr = buffer->GetAsInlinePointer(position, 4, resolve_result);
    int32_t header_size = 0;
    
    if (resolve_result.type == core::PageLocatorResolvement::SinglePage) {
      header_size = nvm::bytes::ToInt32<uint8_t>(
          ptr, 4, result, nvm::bytes::EndianessType::BigEndian);
    } else if (resolve_result.type == core::PageLocatorResolvement::CrossPage) {
      auto raw = buffer->GetAsCopy(position, 4, resolve_result);
      header_size = nvm::bytes::ToInt32<uint8_t>(
          raw->Data(), 4, result, nvm::bytes::EndianessType::BigEndian);
      raw->Destroy();
      
    }

    position += std::streamsize(4);
    return header_size;
  }

  OSMPBF::Blob GetPbfBlob(core::IMemoryBufferAdapter<BlockType>* buffer,
                          const size_t& header_size, const size_t& data_size,
                          std::streampos& position, core::PageLocatorInfo& resolve_result) {
    uint8_t* ptr =
        buffer->GetAsInlinePointer(position, header_size, resolve_result);

    auto blob = OSMPBF::Blob();

    if (resolve_result.type == core::PageLocatorResolvement::SinglePage) {
      blob.ParseFromArray(ptr, header_size);
    } else if (resolve_result.type == core::PageLocatorResolvement::CrossPage) {
      auto raw = buffer->GetAsCopy(position, header_size, resolve_result);
      blob.ParseFromArray(raw->Data(), header_size);
      raw->Destroy();
      
    }

    if (verbose_) {
      std::cout << "Blob { data = " << blob.data_case()
                << ", rawsize = " << blob.raw_size()
                << ", bytesize = " << blob.ByteSizeLong() << " }" << std::endl;
    }

    if (blob.ByteSizeLong() == 0) {
      std::cout << "Blob mismatch :  " << blob.ByteSizeLong() << ":"
                << data_size << std::endl;
      
      return OSMPBF::Blob();
    }

    position += std::streamsize(data_size);
    return std::move(blob);
  }

  OSMPBF::BlobHeader GetPbfHeader(core::IMemoryBufferAdapter<BlockType>* buffer,
                                  const size_t& header_size,
                                  std::streampos& position, core::PageLocatorInfo& resolve_result) {
    uint8_t* ptr =
        buffer->GetAsInlinePointer(position, header_size, resolve_result);

    auto header = OSMPBF::BlobHeader();

    if (resolve_result.type == core::PageLocatorResolvement::SinglePage) {
      header.ParseFromArray(ptr, header_size);
    } else if (resolve_result.type == core::PageLocatorResolvement::CrossPage) {
      auto raw = buffer->GetAsCopy(position, header_size, resolve_result);
      header.ParseFromArray(raw->Data(), header_size);
      raw->Destroy();
      buffer->RemoveBufferPage(resolve_result.end_page_id-1);
    }


    if (verbose_) {
      std::cout << header.type() << " {indexdata = " << header.indexdata()
                << ", datasize = " << header.datasize()
                << ", bytesize = " << header.ByteSizeLong() << "}" << std::endl;
    }

    if (header.ByteSizeLong() == 0) {
      std::cout << "Header : zero-bytes " << std::endl;
      
      return OSMPBF::BlobHeader();
    }

    position += std::streamsize(header_size);
   

    return std::move(header);
  }

  void CleanupBuffer(core::IMemoryBufferAdapter<BlockType>* buffer,const core::PageLocatorInfo& current,const core::PageLocatorInfo& prev){
    if(prev.total_size == 0 && prev.end_page_id == 0 && prev.start_page_id == 0){
      return;
    }
    // std::cout << "page_id: " << prev.start_page_id << ":" << current.start_page_id << std::endl;
    if(prev.start_page_id != current.start_page_id){
      buffer->RemoveBufferPage(prev.start_page_id);
    }
  }

  nvm::Option<PbfBlockMap> Detect(core::IMemoryBufferAdapter<BlockType>* buffer,
                                  const size_t& start) {
    if (!buffer || buffer->StreamSize() == 0) return nvm::Option<PbfBlockMap>();

    // Get header size

    std::streampos position = 0;
    core::PageLocatorInfo resolve_result = core::PageLocatorInfo();
    core::PageLocatorInfo prev_result = core::PageLocatorInfo();

    while (position < buffer->StreamSize()) {
      bool state = false;

      auto header_size = GetHeaderLength(buffer, position,  resolve_result);
      CleanupBuffer(buffer,resolve_result,prev_result);
      prev_result = resolve_result;


      auto header = GetPbfHeader(buffer, header_size, position, resolve_result);
      CleanupBuffer(buffer,resolve_result,prev_result);
      prev_result = resolve_result;

      auto blob =
          GetPbfBlob(buffer, header_size, header.datasize(), position, resolve_result);
      CleanupBuffer(buffer,resolve_result,prev_result);
      prev_result = resolve_result;
     
    }

    std::cout << "\nProcess size: " << position << std::endl;

    return nvm::Option<PbfBlockMap>();
  }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix