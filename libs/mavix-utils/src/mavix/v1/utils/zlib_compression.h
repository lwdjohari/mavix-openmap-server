#pragma once

#include <mavix/v1/core/core.h>
#include <zlib.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

#include "mavix/v1/core/memory_buffer.h"
#include "mavix/v1/core/segment_buffer.h"
#include "mavix/v1/utils/icompression.h"
#include "nvm/strings/readable_bytes.h"
namespace mavix {
namespace v1 {
namespace utils {

using namespace mavix::v1::core;
class ZlibCompression : public ICompression {
 private:
 public:
  ZlibCompression() {}

  ~ZlibCompression() {}

  std::shared_ptr<MemoryBuffer> Inflate(const uint8_t *source,
                                        size_t source_size,
                                        bool &result) override {
    result = false;
    auto zs_ptr = std::make_unique<z_stream>();
    memset(zs_ptr.get(), 0, sizeof(z_stream));

    // RAW ZLIB
    // if (inflateInit2(zs_ptr.get(), -MAX_WBITS) != Z_OK) {
    //   std::ostringstream oss;
    //   oss << "ZLib Inflate Init Exception :" << zs_ptr.get()->msg;
    //   std::cerr << oss.str() << std::endl;
    //   inflateEnd(zs_ptr.get());
    //   return nullptr;
    // }

    // GZIP
    // if (inflateInit2(zs_ptr.get(), 16 + MAX_WBITS) != Z_OK) {
    //   std::ostringstream oss;
    //   oss << "ZLib Inflate Init Exception :" << zs_ptr.get()->msg;
    //   std::cerr << oss.str() << std::endl;
    //   inflateEnd(zs_ptr.get());
    //   return nullptr;
    // }

    // ZLIB
    if (inflateInit(zs_ptr.get()) != Z_OK) {
      std::ostringstream oss;
      oss << "ZLib Inflate Init Exception :" << zs_ptr.get()->msg;

      std::cerr << oss.str() << std::endl;
      inflateEnd(zs_ptr.get());
      return nullptr;
    }

    zs_ptr.get()->next_in =
        const_cast<Bytef *>(reinterpret_cast<const Bytef *>(source));
    zs_ptr.get()->avail_in = static_cast<uInt>(source_size);

    int ret;
    SegmentBuffer buffers(32768);
    MemoryBuffer outbuffer(32768);

    do {
      zs_ptr.get()->next_out = reinterpret_cast<Bytef *>(outbuffer.Data());
      zs_ptr.get()->avail_out = static_cast<uInt>(outbuffer.Size());

      ret = inflate(zs_ptr.get(), Z_NO_FLUSH);
      if (ret == Z_OK || ret == Z_STREAM_END) {
        size_t written = 32768 - zs_ptr->avail_out;
        buffers.Add(outbuffer.Data(), written);
      }
    } while (ret == Z_OK);

    outbuffer.Destroy();
    inflateEnd(zs_ptr.get());

    if (ret != Z_STREAM_END) {
      buffers.Destroy();
      std::ostringstream oss;
      oss << "Zlib Inflate Exception : (" << ret << ") " << zs_ptr.get()->msg;

      std::cerr << oss.str() << std::endl;
      return nullptr;
    }

    auto flat_buffer = buffers.CopyAsMemoryBuffer();

    if (flat_buffer) {
      buffers.Destroy();
      result = true;
    }

    return flat_buffer;
  };

  std::shared_ptr<MemoryBuffer> Deflate(const uint8_t *source, size_t size,
                                        bool &result) override {
    if (!source || size == 0) {
      result = false;
      return nullptr;
    }

    auto zs_ptr = std::make_unique<z_stream>();
    z_stream *zs = zs_ptr.get();
    memset(zs, 0, sizeof(z_stream));

    if (deflateInit(zs, Z_DEFAULT_COMPRESSION) != Z_OK) {
      result = false;
      return nullptr;
    }

    zs->next_in = reinterpret_cast<Bytef *>(const_cast<uint8_t *>(source));
    zs->avail_in = size;

    int ret;
    SegmentBuffer buffers(32768);
    do {
      MemoryBuffer outbuffer(32768);
      zs->next_out = reinterpret_cast<Bytef *>(outbuffer.Data());
      zs->avail_out = outbuffer.Size();

      ret = deflate(zs, Z_FINISH);
      if (ret == Z_OK || ret == Z_STREAM_END) {
        buffers.Add(std::move(outbuffer));
        // compressed->insert(compressed->end(), outbuffer.begin(),
        //                    outbuffer.begin() + (32768 - zs->avail_out));
      } else {
        outbuffer.Destroy();
      }
    } while (ret == Z_OK);

    deflateEnd(zs);

    if (ret != Z_STREAM_END) {
      result = false;
      return nullptr;

      // std::ostringstream oss;
      // oss << "Exception during compression: (" << ret << ") " << zs.msg;
      // throw std::runtime_error(oss.str());
    }

    result = true;

    auto flat_buffer = buffers.CopyAsMemoryBuffer();
    buffers.Destroy();
    return std::move(flat_buffer);
  };

  std::shared_ptr<MemoryBuffer> Deflate(
      std::shared_ptr<MemoryBuffer> buffer) override {
    bool result;
    return Deflate(buffer->Data(), buffer->Size(), result);
  }

  std::shared_ptr<MemoryBuffer> Inflate(
      std::shared_ptr<MemoryBuffer> buffer) override {
    bool result;
    return Inflate(buffer->Data(), buffer->Size(), result);
  }
};

}  // namespace utils
}  // namespace v1
}  // namespace mavix