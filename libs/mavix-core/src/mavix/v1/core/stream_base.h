#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <filesystem>
#include <string>

#include <mavix/v1/core/stream_state.h>

namespace mavix {
namespace v1 {
namespace core {

class IStreamBase {
  virtual std::string GetFilename() const = 0;

  virtual std::string GetDirectoryPath() const = 0;

  virtual std::string GetDirectorySeparatorPath() const = 0;

  virtual std::string GetFileExtension() const = 0;

  virtual StreamState Open() = 0;

  virtual StreamState Close() = 0;

  virtual const std::string& File() const = 0;

  virtual std::streampos CurrentPosition() = 0;

  virtual std::streampos MoveTo(std::streampos pos) = 0;

  virtual std::streampos Next(std::streamsize size) = 0;

  virtual std::streampos Prev(std::streamsize size) = 0;

  virtual std::streampos Next() = 0;

  virtual std::streampos Prev() = 0;

  virtual bool IsOpen() const = 0;

  virtual bool IsEof() const = 0;

  virtual bool IsGood() const = 0;

  virtual std::streamsize Size() const = 0;
};

class StreamBase : public IStreamBase {
 private:
  std::string filename_;
  std::filesystem::path path_;

 protected:
  explicit StreamBase(const std::string& path)
      : path_(std::string(path)), filename_(std::string(path)) {}

 public:
  virtual ~StreamBase() {}
  std::string GetFilename() const override {
    return path_.filename().string();
    ;
  };

  std::string GetDirectoryPath() const override {
    return path_.filename().string();
  };

  std::string GetDirectorySeparatorPath() const override {
    return std::string(1, std::filesystem::path::preferred_separator);
  };

  std::string GetFileExtension() const override {
    return path_.extension().string();
  };
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
