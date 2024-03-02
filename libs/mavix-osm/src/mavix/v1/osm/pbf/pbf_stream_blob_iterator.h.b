
#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "mavix/v1/core/stream.h"
#include "mavix/v1/core/shared_buffer.h"
#include "nvm/bytes/byte.h"
#include "osmpbf/osmpbf.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

/// @brief  Split OSM bytes stream into blob defined and iterate through all raw
/// blob in OSM PBF stream.
///         This class is designed with thread-safe in mind when iterating
///         through next() method, Otherwise please consider careful when use
///         other methods other than mentioned above.
/// @author Linggawasistha Djohari <linggawasistha.djohari@outlook.com>
class PbfStreamBlobIterator {
 private:
  size_t blockCount_;
  core::Stream &stream_;
  std::mutex mutex_;
  

  /// @brief Implementation for decoding and iterated to next OSM raw blob.
  /// @return std::shared_ptr<core::SharedBuffer> of OSM PBF Raw blob
  ///         otherwise return nullptr if eof, error, partial read or no read
  ///         happen.
  std::shared_ptr<core::SharedBuffer> GetNextBlob();

 public:
  /// @brief Construct object with reference to backing stream of OSM PBF Binary
  /// Stream.
  /// @param stream Reference to nvm::stream of OSM PBF binary stream.
  explicit PbfStreamBlobIterator(core::Stream &stream);
  virtual ~PbfStreamBlobIterator();

  /// @brief Get the backing nvm::Stream object for OSM PBF Stream.
  /// @return 
  core::Stream &Stream();

  /// @brief Set the stream position to beginning of stream.
  void Begin();

  /// @brief  Decode bytes stream to OSMPBF::BlobHeader and move the ownership
  /// to the caller.
  ///         This function also move the stream position to successful read
  ///         position. This method alone is not thread-safe, lock only
  ///         implemented on next() method.
  /// @param headerLength number of bytes of raw header
  /// @return 
  ///         otherwise return nullptr if error, partial read or no read happen.
  std::shared_ptr<OSMPBF::BlobHeader> ReadHeader(const size_t &headerLength);

  /// @brief  Read, copy stream of bytes and move the ownership to the caller.
  ///         This method alone is not thread-safe, lock only implemented on
  ///         next() method.
  /// @param blobHeader Reference to OSMPBF::BlobHeader
  /// @return 
  ///         otherwise return nullptr if error, partial read or no read happen.
  std::shared_ptr<core::SharedBuffer> ReadRawBlob(
      OSMPBF::BlobHeader &blobHeader);

  /// @brief  Decode and return OSM PBF Raw Blob bytes.
  ///         This function move the new position after sucessful decoding.
  ///         This method is thread-safe and utilizing lock-guard inside it.
  /// @return std::shared_ptr<core::SharedBuffer> of OSM PBF Raw blob
  ///         otherwise return nullptr if eof, error, partial read or no read
  ///         happen.
  std::shared_ptr<core::SharedBuffer> Next();
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1

}  // namespace mavix
