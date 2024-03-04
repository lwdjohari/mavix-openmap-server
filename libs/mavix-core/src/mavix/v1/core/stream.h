#pragma once

#include <mavix/v1/core/core.h>

#include <fstream>

#include "mavix/v1/core/memory_buffer.h"
#include "nvm/macro.h"

namespace mavix {
namespace v1 {
namespace core {

enum class StreamState {
  Ok = 0,
  Error = 1,
  FileNotExist = 2,
  IndexOutOfBound = 3,
  PermissionFailed = 4,
  AlreadyOpen = 5,
  Processing = 6,
  Stoped = 7
};

NVM_ENUM_CLASS_DISPLAY_TRAIT(StreamState)

class Stream {
 private:
  std::ifstream stream_;
  std::streamsize size_;
  std::string file_;

 public:
  explicit Stream(const std::string& file)
      : stream_(nullptr), size_(std::streamsize()), file_(std::string(file)) {}
  ~Stream() {}

  StreamState Open() {
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

  StreamState Close() {
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

  const std::string& File() const { return file_; }

  std::streampos CurrentPosition() {
    if (!stream_.good() || !stream_.is_open()) return std::streampos(-1);

    return stream_.tellg();
  }

  template <typename T>
  bool CopyToPointer(uint8_t* dest, std::streampos pos, size_t size) {
    if (!dest || !stream_.good() || !stream_.is_open() || size == 0) return false;

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

  template <typename T>
  std::shared_ptr<core::MemoryBuffer<T>> CopyToSharedBuffer(T id,
                                                            std::streampos pos,
                                                            size_t size) {
    if (!stream_.good() || !stream_.is_open() || size == 0) return nullptr;

    if (IsOutOfBound(pos, size)) return nullptr;

    if (!stream_.seekg(pos)) {
      stream_.clear();
      return nullptr;
    }

    auto buffer =
        core::memory::make_shared_with_allocator<core::MemoryBuffer<T>>(id,
                                                                        size);

    if (!stream_.read(reinterpret_cast<char*>(buffer->Data()), size)) {
      stream_.clear();
      return nullptr;
    }

    // cppcheck-suppress returnStdMoveLocal
    return std::move(buffer);
  }

  std::streampos MoveTo(std::streampos pos) {
    if (!stream_.good() || !stream_.is_open()) return std::streampos(-1);
    if (pos >= size_ || pos < 0) return std::streampos(-1);

    if (!stream_.seekg(pos)) {
      stream_.clear();
      return std::streampos(-1);
    }

    return pos;
  }

  std::streampos Next(std::streamsize size) {
    if (!stream_.good() || !stream_.is_open()) return std::streampos(-1);

    auto newPos = CurrentPosition() + size;
    if (newPos >= size_) return std::streampos(-1);

    if (!stream_.seekg(newPos, std::ios::beg)) {
      stream_.clear();
      return std::streampos(-1);
    }

    return newPos;
  }

  std::streampos Prev(std::streamsize size) {
    if (!stream_.good() || !stream_.is_open()) return std::streampos(-1);

    auto newPos = CurrentPosition() - size;
    if (newPos <= 0) return std::streampos(-1);

    if (!stream_.seekg(newPos, std::ios::beg)) {
      stream_.clear();
      return std::streampos(-1);
    }

    return newPos;
  }

  std::streampos Next() { return Next(CurrentPosition() + std::streampos(1)); }

  std::streampos Prev() { return Next(CurrentPosition() - std::streampos(1)); }

  bool IsOutOfBound(const std::streampos& pos, const size_t& size) {
    return (pos + static_cast<std::streampos>(size) > size_) ? true : false;
  }

  bool IsOpen() const { return stream_.is_open(); }

  bool IsEof() const { return stream_.eof(); }

  bool IsGood() const { return stream_.good(); }

  std::streamsize Size() const { return size_; }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix