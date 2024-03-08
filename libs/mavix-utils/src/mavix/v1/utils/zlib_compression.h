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
    auto zs_ptr = std::make_unique<z_stream>();
    z_stream *zs = zs_ptr.get();
    memset(zs, 0, sizeof(z_stream));

    if (inflateInit(zs) != Z_OK) {
      inflateEnd(zs);
      result = false;
      return nullptr;
    }

    // zs->next_in = reinterpret_cast<Bytef *>(const_cast<uint8_t *>(source));
    // zs->avail_in = source_size;

    // int ret;
    // SegmentBuffer buffers(32768);
    // do {
    //   MemoryBuffer outbuffer(32768);
    //   zs->next_out = reinterpret_cast<Bytef *>(outbuffer.Data());
    //   zs->avail_out = outbuffer.Size();

    //   ret = inflate(zs, 0);
    //   if (ret == Z_OK || ret == Z_STREAM_END) 
    //   {
    //     std::cout << "INflate success" << std::endl;

    //     buffers.Add(std::move(outbuffer));
    //     // decompressed->insert(decompressed->end(), outbuffer.Begin(),
    //     //                      outbuffer.Begin() + (32768 - zs->avail_out));
    //   } else {
    //     std::cout << "INflate failed" << std::endl;
    //     outbuffer.Destroy();
    //   }

    // } while (ret == Z_OK);

    // inflateEnd(zs);

    // if (ret != Z_STREAM_END) {
    //   result = false;
    //   return nullptr;
    //   // std::ostringstream oss;
    //   // oss << "Exception during decompression: (" << ret << ") " << zs.msg;
    //   // throw std::runtime_error(oss.str());
    // }

    // result = true;

    // auto flat_buffer = buffers.GetAsMemoryBuffer();

    // std::cout << "BUFFER:" << ((!flat_buffer) ? 0 : flat_buffer->Size())
    //           << std::endl;
    // if (flat_buffer) buffers.Destroy();
    return nullptr;
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

    auto flat_buffer = buffers.GetAsMemoryBuffer();
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