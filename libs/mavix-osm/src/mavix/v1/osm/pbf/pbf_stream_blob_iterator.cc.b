#include "mavix/v1/osm/pbf/pbf_stream_blob_iterator.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

PbfStreamBlobIterator::PbfStreamBlobIterator(core::Stream &stream)
    : stream_(stream), blockCount_(0) {}

PbfStreamBlobIterator::~PbfStreamBlobIterator() {}

// Private Method
std::shared_ptr<core::SharedBuffer> PbfStreamBlobIterator::GetNextBlob() {
  // return if already reach eof
  if (stream_.IsEof()) return nullptr;

  size_t length = 0;
  size_t successfulRead = 0;

  // Get header size
  uint8_t lengthBytes[4];

  auto resBytes = stream_.copyToPointer(lengthBytes, 4, successfulRead);
  if (resBytes != nvm::StreamState::Success) return nullptr;

  length = conv_.decodeInt32(lengthBytes, 4, decodeErr, true);
  if (decodeErr != nvm::bconf_err_t::BCONV_ERR_OK) return nullptr;

  // Move the cursor, int32 is 4 bytes.
  stream_.Next(4, nvm::StreamCursorRelativePosition::CurrentPosition);

  if (length == 0) return nullptr;

  // Found new block
  blockCount_++;

  // Copy the bytes
  auto blobHeader = ReadHeader(length);
  auto rawBinary = ReadRawBlob(*(blobHeader));

  // Return the stream buffer
  return std::move(rawBinary);
}

// Public Method
core::Stream &PbfStreamBlobIterator::Stream() { return stream_; }

void PbfStreamBlobIterator::Begin() {
  blockCount_ = 0;
  stream_.MoveTo(0);
}

std::shared_ptr<OSMPBF::BlobHeader> PbfStreamBlobIterator::ReadHeader(
    const size_t &headerLength) {
  if (!stream_.IsOpen()) return nullptr;

  size_t successfulRead = 0;
  auto buff = std::make_shared<core::SharedBuffer<std::string>>("orh", headerLength);
  auto res = stream_.copyToPointer(buff->pointerToData(0), headerLength,
                                   successfulRead);

  // Must be full read, otherwise indicated corrupted file or else.
  if (res != nvm::StreamState::Success) return nullptr;

  // Move the cursor
  stream_.move(headerLength,
               nvm::StreamCursorRelativePosition::CurrentPosition);

  return std::move(std::shared_ptr<OSMPBF::BlobHeader>());
}

std::shared_ptr<nvm::StreamBuffer> PbfStreamBlobIterator::ReadRawBlob(
    OSMPBF::BlobHeader &blobHeader) {
  auto rawSize = blobHeader.ByteSizeLong();

  if (rawSize == 0) return nullptr;

  size_t successfulRead = 0;
  auto buff = std::make_shared<nvm::StreamBuffer>("ord", rawSize);
  auto res =
      stream_.copyToPointer(buff->pointerToData(0), rawSize, successfulRead);

  // Must be full read, otherwise indicated corrupted file or else.
  if (res != nvm::StreamState::Success) return nullptr;

  // Move the cursor
  stream_.move(rawSize, nvm::StreamCursorRelativePosition::CurrentPosition);

  return std::move(std::shared_ptr<nvm::StreamBuffer>());
}

std::shared_ptr<nvm::StreamBuffer> PbfStreamBlobIterator::next() {
  // Make sure this operation is thread safe
  // Add lock guard to avoid UB
  std::lock_guard<std::mutex> lock(mutex_);
  return std::move(getNextBlob());
}

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix
