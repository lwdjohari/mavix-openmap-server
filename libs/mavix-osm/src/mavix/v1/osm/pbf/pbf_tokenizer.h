#pragma once

#include <mavix/v1/core/core.h>

#include <fstream>
#include <memory>

#include "mavix/v1/osm/pbf/pbf_declare.h"
#include "mavix/v1/core/stream_buffer.h"
#include "mavix/v1/osm/block_type.h"
#include "mavix/v1/osm/pbf/pbf_block_map.h"
#include "nvm/bytes/byte.h"
#include "nvm/option.h"
#include "nvm/strings/readable_bytes.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

using namespace nvm;
using namespace mavix::v1::core;
using namespace nvm::bytes;

struct PbfTokenizerErr {
  std::streampos position;
  StreamState state;
  bool* skipped;

  PbfTokenizerErr()
      : position(0), state(StreamState::Unknown), skipped(nullptr) {}

  PbfTokenizerErr(std::streampos position, StreamState state, bool* skipped)
      : position(position), state(state), skipped(skipped) {}
};

class PbfTokenizer;

typedef void (*PbfTokenizerOnPbfDataReady)(PbfTokenizer* sender,
                                           std::shared_ptr<PbfBlobData> data);
typedef void (*PbfTokenizerOnError)(PbfTokenizer* sender,
                                    std::shared_ptr<PbfTokenizerErr> err);
typedef void (*PbfTokenizerOnStart)(PbfTokenizer* sender);
typedef void (*PbfTokenizerOnFinish)(PbfTokenizer* sender);

class PbfTokenizer {
 private:
  bool verbose_;
  IMemoryBufferAdapter* buffer_;
  PbfTokenizerOnError callback_err_;
  PbfTokenizerOnPbfDataReady callback_data_ready_;
  PbfTokenizerOnStart callback_started_;
  PbfTokenizerOnFinish callback_finished_;

 protected:
  void RaiseOnDataReady(std::shared_ptr<PbfBlobData> data, bool& raised) {
    if (!callback_data_ready_){ 
      raised = false;
      return;
    }

    callback_data_ready_(this, std::move(data));
    raised = true;
  }

  void RaiseOnErr(std::shared_ptr<PbfTokenizerErr> err) {
    if (!callback_err_) return;

    callback_err_(this, std::move(err));
  }

  void RaiseOnStarted() {
    if (!callback_started_) return;

    callback_started_(this);
  }

  void RaiseOnFinished() {
    if (!callback_finished_) return;

    callback_finished_(this);
  }

 public:
  explicit PbfTokenizer(IMemoryBufferAdapter* buffer, bool verbose = true)
      : buffer_(buffer),
        verbose_(verbose),
        callback_err_(nullptr),
        callback_data_ready_(nullptr),
        callback_started_(nullptr),
        callback_finished_(nullptr){};
  ~PbfTokenizer(){};

  void OnDataReady(PbfTokenizerOnPbfDataReady callback) {
    callback_data_ready_ = callback;
  }

  void OnDataReadyUnregister() { callback_data_ready_ = nullptr; }

  void OnDataError(PbfTokenizerOnError callback) { callback_err_ = callback; }

  void OnDataErrorUnregister() { callback_data_ready_ = nullptr; }

  void OnStarted(PbfTokenizerOnStart callback) { callback_started_ = callback; }

  void OnStartedUnregister() { callback_started_ = nullptr; }

  void OnFinished(PbfTokenizerOnFinish callback) {
    callback_started_ = callback;
  }

  void OnFinishedUnregister() { callback_started_ = nullptr; }

  int32_t GetHeaderLength(std::streampos& position, PageLocatorInfo& result) {
    ByteOpResult byte_result;

    uint8_t* ptr = buffer_->GetAsInlinePointer(position, 4, result);
    int32_t header_size = 0;

    if (result.type == PageLocatorResolvement::SinglePage) {
      header_size =
          ToInt32<uint8_t>(ptr, 4, byte_result, EndianessType::BigEndian);
    } else if (result.type == PageLocatorResolvement::CrossPage) {
      auto raw = buffer_->GetAsCopy(position, 4, result);
      header_size = ToInt32<uint8_t>(raw->Data(), 4, byte_result,
                                     EndianessType::BigEndian);
      raw->Destroy();
    }

    position += std::streamsize(4);
    return header_size;
  }

  OSMPBF::Blob GetPbfBlob(const size_t& header_size, const size_t& data_size,
                          std::streampos& position, PageLocatorInfo& result) {
    uint8_t* ptr = buffer_->GetAsInlinePointer(position, header_size, result);

    auto blob = OSMPBF::Blob();

    if (result.type == PageLocatorResolvement::SinglePage) {
      blob.ParseFromArray(ptr, header_size);
    } else if (result.type == PageLocatorResolvement::CrossPage) {
      auto raw = buffer_->GetAsCopy(position, header_size, result);
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

    position += std::streamsize(blob.ByteSizeLong());
    return std::move(blob);
  }

  std::pair<std::streampos,std::streamsize> GetRawBufferPosition( const OSMPBF::BlobHeader &blob_header, const OSMPBF::Blob &blob, const  std::streampos &position){
    std::streamsize size = std::streamsize( blob_header.datasize() - blob.ByteSizeLong());
    std::streampos new_position = position;;

    return {new_position, size};
  }

  std::shared_ptr<MemoryBuffer> GetRawBuffer( std::streampos&position, const size_t& size, PageLocatorInfo &result){
    return buffer_->GetAsCopy(position,size,result);
  }

  OSMPBF::BlobHeader GetPbfHeader(const size_t& header_size,
                                  std::streampos& position,
                                  PageLocatorInfo& result) {
    uint8_t* ptr = buffer_->GetAsInlinePointer(position, header_size, result);

    auto header = OSMPBF::BlobHeader();

    if (result.type == PageLocatorResolvement::SinglePage) {
      header.ParseFromArray(ptr, header_size);
    } else if (result.type == PageLocatorResolvement::CrossPage) {
      auto raw = buffer_->GetAsCopy(position, header_size, result);
      header.ParseFromArray(raw->Data(), header_size);
      raw->Destroy();
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

  void CleanupBuffer(const PageLocatorInfo& current,
                     const PageLocatorInfo& prev) {
    if (prev.total_size == 0 && prev.end_page_id == 0 &&
        prev.start_page_id == 0) {
      return;
    }

    if (prev.start_page_id != current.start_page_id) {
      buffer_->RemoveBufferPage(prev.start_page_id);
    }
  }

  nvm::Option<PbfBlockMap> Split() {
    if (!buffer_ || buffer_->Size() == 0) return nvm::Option<PbfBlockMap>();

    // Get header size
    size_t blob_count = 0;
    size_t header_count = 0;

    std::streampos position = 0;
    PageLocatorInfo result = PageLocatorInfo();
    PageLocatorInfo prev_result = PageLocatorInfo();

    while (position < buffer_->Size()) {
      bool state = false;

      auto header_size = GetHeaderLength(position, result);
      CleanupBuffer(result, prev_result);
      prev_result = result;

      auto header = GetPbfHeader(header_size, position, result);
      CleanupBuffer(result, prev_result);
      prev_result = result;
      header_count++;

      auto blob = GetPbfBlob(header_size, header.datasize(), position, result);
      CleanupBuffer(result, prev_result);
      prev_result = result;

      auto raw_pos =  GetRawBufferPosition(header,blob,position);
      position = raw_pos.first + raw_pos.second;

      auto raw_buffer = GetRawBuffer(raw_pos.first ,raw_pos.second,result);
      
      bool raised = false;
      auto data = std::make_shared<PbfBlobData>(header,blob,raw_buffer);
      RaiseOnDataReady(data,raised);
      if(!raised){
        data->blob_data->Destroy();
      }
      
      blob_count++;
    }
#if defined(MAVIX_DEBUG_CORE)
    std::cout << "Pbf Tokenizer : { header: " << header_count
              << " , blob: " << blob_count << " }" << std::endl;
    std::cout << "Pbf Processing size: "
              << strings::ConvertBytesToReadableSizeString(
                     std::streamsize(position))
              << std::endl;
#endif
    return nvm::Option<PbfBlockMap>();
  }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix