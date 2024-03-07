#pragma once

#include <mavix/v1/core/core.h>

#include <fstream>

#include "mavix/v1/core/stream_base.h"
#include "mavix/v1/core/memory_buffer.h"
#include "nvm/macro.h"
namespace mavix {
namespace v1 {
namespace core {



class Stream : public StreamBase, public ICacheBucketBuffer {
 private:
  std::ifstream stream_;
  std::streamsize size_;
  std::string file_;

 public:
  explicit Stream(const std::string& file)
      : StreamBase(file),
        stream_(nullptr),
        size_(std::streamsize()),
        file_(std::string(file)) {}
  ~Stream() {}

  StreamState Open() override {
    if (stream_.is_open()) return StreamState::AlreadyOpen;
    stream_ = std::ifstream(file_, std::ios::binary | std::ios::ate);

    if (!stream_.is_open()) {
      std::cerr << "Failed to open file: " << file_ << std::endl;
      size_ = std::streamsize(-1);
      return StreamState::Error;
    }

    size_ = stream_.tellg();
    stream_.seekg(0, std::ios::beg);

    return StreamState::Ok;
  }

  StreamState Close() override {
    if (!stream_.is_open()) {
      return StreamState::Error;
    }

    stream_.close();

    if (stream_.failbit == std::ios_base::iostate::_S_failbit) {
      stream_.clear();
      return StreamState::Error;
    }

    return StreamState::Ok;
  }

  const std::string& File() const override { return file_; }

  std::streampos CurrentPosition() override {
    if (!stream_.good() || !stream_.is_open()) return std::streampos(-1);

    return stream_.tellg();
  }

 
  bool CopyToPointer(uint8_t* dest, std::streampos pos, size_t size) override {
    if (!dest || !stream_.good() || !stream_.is_open() || size == 0)
      return false;

    if (IsOutOfBound(pos, size)) return false;

    if (!stream_.seekg(pos)) {
      stream_.clear();
      return false;
    }

    if (!stream_.read(reinterpret_cast<char*>(dest), size)) {
      stream_.clear();
      return false;
    }

    return true;
  }

 
  std::shared_ptr<core::MemoryBuffer> CopyToSharedBuffer(
                                                            std::streampos pos,
                                                            size_t size) {
    if (!stream_.good() || !stream_.is_open() || size == 0) return nullptr;

    if (IsOutOfBound(pos, size)) return nullptr;

    if (!stream_.seekg(pos)) {
      stream_.clear();
      return nullptr;
    }

    auto buffer = std::make_shared<core::MemoryBuffer>( size);

    if (!stream_.read(reinterpret_cast<char*>(buffer->Data()), size)) {
      stream_.clear();
      return nullptr;
    }

    // cppcheck-suppress returnStdMoveLocal
    return std::move(buffer);
  }

  std::streampos MoveTo(std::streampos pos) override{
    if (!stream_.good() || !stream_.is_open()) return std::streampos(-1);
    if (pos >= size_ || pos < 0) return std::streampos(-1);

    if (!stream_.seekg(pos)) {
      stream_.clear();
      return std::streampos(-1);
    }

    return pos;
  }

  std::streampos Next(std::streamsize size) override{
    if (!stream_.good() || !stream_.is_open()) return std::streampos(-1);

    auto newPos = CurrentPosition() + size;
    if (newPos >= size_) return std::streampos(-1);

    if (!stream_.seekg(newPos, std::ios::beg)) {
      stream_.clear();
      return std::streampos(-1);
    }

    return newPos;
  }

  std::streampos Prev(std::streamsize size) override {
    if (!stream_.good() || !stream_.is_open()) return std::streampos(-1);

    auto newPos = CurrentPosition() - size;
    if (newPos <= 0) return std::streampos(-1);

    if (!stream_.seekg(newPos, std::ios::beg)) {
      stream_.clear();
      return std::streampos(-1);
    }

    return newPos;
  }

  std::streampos Next()override { return Next(CurrentPosition() + std::streampos(1)); }

  std::streampos Prev() override{ return Next(CurrentPosition() - std::streampos(1)); }

  bool IsOutOfBound(const std::streampos& pos, const size_t& size) {
    return (pos + static_cast<std::streampos>(size) > size_) ? true : false;
  }

  bool IsOpen() const override { return stream_.is_open(); }

  bool IsEof() const override { return stream_.eof(); }

  bool IsGood() const override { return stream_.good(); }

  std::streamsize Size() const override { return size_; }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix