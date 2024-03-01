#include "mavix/v1/utils/zlib_compression.h"

namespace mavix {
namespace v1 {
namespace utils {

ZlibCompression::ZlibCompression() {}

ZlibCompression::~ZlibCompression() {}

std::shared_ptr<core::SharedBuffer> ZlibCompression::deflateBytes(
    std::shared_ptr<core::SharedBuffer> buffer) {
  bool result;

  this->deflateBytes(buffer->Ptr(0), buffer->Size(), nullptr, result);

  if (!result) return nullptr;

  auto s = std::make_shared<core::SharedBuffer>(buffer->Id(), buffer->Size());

  return std::move(s);
}

void ZlibCompression::deflateBytes(const uint8_t *source, size_t sourceLength,
                                   std::vector<uint8_t> *compressed,
                                   bool &result) {
  if (!compressed) {
    result = false;
    return;
  }

  auto zs_ptr = std::make_unique<z_stream>();
  z_stream *zs = zs_ptr.get();
  memset(zs, 0, sizeof(z_stream));

  if (deflateInit(zs, Z_DEFAULT_COMPRESSION) != Z_OK) {
    result = false;
    return;
  }

  zs->next_in = reinterpret_cast<Bytef *>(const_cast<uint8_t *>(source));
  zs->avail_in = sourceLength;

  int ret;
  do {
    std::vector<uint8_t> outbuffer(32768);
    zs->next_out = reinterpret_cast<Bytef *>(outbuffer.data());
    zs->avail_out = outbuffer.size();

    ret = deflate(zs, Z_FINISH);
    if (ret == Z_OK || ret == Z_STREAM_END) {
      compressed->insert(compressed->end(), outbuffer.begin(),
                         outbuffer.begin() + (32768 - zs->avail_out));
    }
  } while (ret == Z_OK);

  deflateEnd(zs);

  if (ret != Z_STREAM_END) {
    result = false;
    return;

    // std::ostringstream oss;
    // oss << "Exception during compression: (" << ret << ") " << zs.msg;
    // throw std::runtime_error(oss.str());
  }

  result = true;
}

std::shared_ptr<core::SharedBuffer> ZlibCompression::inflateBytes(
    std::shared_ptr<core::SharedBuffer> buffer) {
  return std::shared_ptr<core::SharedBuffer>();
}

void ZlibCompression::inflateBytes(const uint8_t *source, size_t sourceLength,
                                   std::vector<uint8_t> *decompressed,
                                   bool &result) {
  auto zs_ptr = std::make_unique<z_stream>();
  z_stream *zs = zs_ptr.get();
  memset(zs, 0, sizeof(z_stream));

  if (inflateInit(zs) != Z_OK) {
    result = false;
    return;
  }

  zs->next_in = reinterpret_cast<Bytef *>(const_cast<uint8_t *>(source));
  zs->avail_in = sourceLength;

  int ret;
  do {
    std::vector<uint8_t> outbuffer(32768);
    zs->next_out = reinterpret_cast<Bytef *>(outbuffer.data());
    zs->avail_out = outbuffer.size();

    ret = inflate(zs, 0);
    if (ret == Z_OK || ret == Z_STREAM_END) {
      decompressed->insert(decompressed->end(), outbuffer.begin(),
                           outbuffer.begin() + (32768 - zs->avail_out));
    }
  } while (ret == Z_OK);

  inflateEnd(zs);

  if (ret != Z_STREAM_END) {
    result = false;
    return;
    // std::ostringstream oss;
    // oss << "Exception during decompression: (" << ret << ") " << zs.msg;
    // throw std::runtime_error(oss.str());
  }

  result = true;
}

}  // namespace utils
}  // namespace v1
}  // namespace mavix