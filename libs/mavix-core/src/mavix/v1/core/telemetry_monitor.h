#pragma once

#include <mavix/v1/core/core.h>

#include <memory>
#include <thread>

#include "absl/container/flat_hash_set.h"
#include "absl/container/node_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"

namespace mavix {
namespace v1 {
namespace core {

class BasicTelemetryItem {
 public:
  BasicTelemetryItem(){};
  virtual ~BasicTelemetryItem(){};
};

template <typename T>
class TelemetryItem : public BasicTelemetryItem {
 private:
  std::string key_;
  T value_;

 public:
  TelemetryItem() : BasicTelemetryItem(), key_(), value_() {}

  TelemetryItem(const std::string &key, T value)
      : BasicTelemetryItem(), value_(value), key_(std::string(key)) {}

  TelemetryItem(const std::string &key, T &&value)
      : BasicTelemetryItem(),
        value_(std::forward<T>(value)),
        key_(std::string(key)) {}

  ~TelemetryItem() {}

  const T &Refval() const { return value_; }

  const T Value() const { return value_; }

  T &Refval() { return value_; }

  T Value() { return value_; }

  void Value(T value) { value_ = value; }

  void Value(T &&value) { value_ = std::forward<T>(value); }
};

class TelemetryStorage {
 private:
  absl::node_hash_map<std::string, BasicTelemetryItem> storage_;
  absl::flat_hash_set<std::string> storage_keys_;
  mutable absl::Mutex mu_;

 public:
  TelemetryStorage() = default;

  template <typename T>
  bool Add(const std::string &key, T &&item) {
    absl::MutexLockMaybe lock(&mu_);
    if (!Contains(key)) return false;

    storage_keys_.emplace(key);
    storage_.emplace(std::forward<T>(item));

    return true;
  }

  template <typename T>
  T &Item(const std::string &key) const {
    absl::MutexLockMaybe lock(&mu_);

    if (!Contains(key)) return nullptr;

    return static_cast<T>(storage_.at(key));
  }

  absl::node_hash_map<std::string, BasicTelemetryItem> &Items() {
    absl::MutexLockMaybe lock(&mu_);

    return storage_;
  }

  bool Remove(const std::string &key) {
    absl::MutexLockMaybe lock(&mu_);

    if (!Contains(key)) return false;
    storage_keys_.erase(key);
    return storage_.erase(key);
  }

  void Clear() {
    absl::MutexLockMaybe lock(&mu_);

    storage_keys_.clear();
    storage_.clear();
  }

  size_t Size() const {
    absl::MutexLockMaybe lock(&mu_);
    return storage_.size();
  }

  bool Contains(const std::string &key) const {
    absl::MutexLockMaybe lock(&mu_);

    return storage_keys_.contains(key);
  }
};

class TelemetryMonitor {
 private:
  mutable absl::Mutex mu_;
  TelemetryStorage storage_global_;
  TelemetryStorage storage_last_interval_;
  std::thread worker_;
 public:
  TelemetryMonitor(){};
  ~TelemetryMonitor(){};

 public:
  const TelemetryStorage &GetLastInterval() const {
    absl::MutexLockMaybe lock(&mu_);
    return storage_last_interval_;
  }

  TelemetryStorage &GetLastInterval() {
    absl::MutexLockMaybe lock(&mu_);
    return storage_last_interval_;
  }

  void Start() { absl::MutexLock lock(&mu_); }

  void Stop() { absl::MutexLock lock(&mu_); }

  void Reset() { absl::MutexLock lock(&mu_); }
};

}  // namespace core
}  // namespace v1
}  // namespace mavix
