#pragma once

#include <mavix/v1/core/core.h>

#include <fstream>

namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

class PbfBlockMap {
 private:
  std::streampos header_start_;
  std::streampos raw_pbf_start_;
  size_t header_size_;
  size_t raw_pbf_size_;

 public:
  PbfBlockMap(const std::streampos &header_start, const size_t &header_size,
              const std::streampos &raw_start, const size_t &raw_size)
      : header_size_(size_t(header_size)),
        raw_pbf_size_(size_t(raw_size)),
        raw_pbf_start_(std::streampos(header_start)),
        header_start_(std::streampos(raw_size)){

        };
        
  ~PbfBlockMap(){

  };

  bool IsEmpty() const {
    return (header_size_ == 0 && raw_pbf_size_ == 0) ? true : false;
  }

  bool IsHeaderSet() { return (header_size_ == 0) ? false : true; }

  bool IsRawBinarySet() { return (raw_pbf_size_ == 0) ? false : true; }

  std::streampos HeaderSize() const { return header_size_; }

  std::streampos RawBinarySize() const { return raw_pbf_size_; }

  std::streampos HeaderPosition() const { return header_start_; }

  std::streampos RawBinaryPosition() const { return raw_pbf_start_; }

  size_t BlockSize() const { return header_size_ + raw_pbf_size_; }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix