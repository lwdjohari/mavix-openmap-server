#pragma once

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
  Stoped = 7,
  Unknown = 8
};

NVM_ENUM_CLASS_DISPLAY_TRAIT(StreamState)
}  // namespace core
}  // namespace v1
}  // namespace mavix
