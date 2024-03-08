#pragma once
#include <mavix/v1/core/core.h>

#include <memory>

#include "absl/container/node_hash_map.h"
#include "mavix/v1/osm/element_type.h"

namespace mavix {
namespace v1 {
namespace osm {

using namespace mavix::v1::osm;

enum class KnownPropertyType {
  Unknown = 0,
  Timestamp = 1,
  String = 2,
  Int32 = 3,
  Uint32 = 4,
  Int64 = 5,
  Uint64 = 6,
  Float = 7,
  Double = 8,
  Others = 9,
};

class BasicElementProperty {
 private:
  KnownPropertyType propertyType_;
  std::string type_;

 protected:
 public:
  BasicElementProperty()
      : propertyType_(KnownPropertyType::Unknown), type_(std::string()){};
  BasicElementProperty(KnownPropertyType prop_type, const std::string &type)
      : propertyType_(prop_type), type_(std::string(type)){};
  ~BasicElementProperty(){};

  KnownPropertyType PropertyType() { return propertyType_; }

  std::string Type() { return type_; }
};

template <typename T>
class ElementProperty : public BasicElementProperty {
 private:
  T val_;

 public:
  ElementProperty(T value, KnownPropertyType prop_type, const std::string &type)
      : BasicElementProperty(prop_type,type), val_(value) {}

  T Value() { return val_; }

  ~ElementProperty() {}
};

class ElementBase {
 private:
  int64_t id_;
  absl::node_hash_map<std::string, BasicElementProperty> properties_;
  ElementType type_;

 protected:
  ElementBase() : id_(0), properties_(), type_(ElementType::Unknown){};
  ElementBase(int64_t id, ElementType type,
              absl::node_hash_map<std::string, BasicElementProperty> tags)
      : id_(id), properties_(tags), type_(type){};

 public:
  
  virtual ~ElementBase(){};

  constexpr static double COORDINATE_SCALING_FACTOR = 0.000000001;
  int64_t Id() const { return id_; }
  ElementType Type() const { return type_; }
  absl::node_hash_map<std::string, BasicElementProperty> Tags() const {
    return properties_;
  }

  bool HasTags() { return !properties_.empty(); }

  bool HasTag(const std::string &key) { return properties_.contains(key); }

  template <typename T>
  T GetTag(const std::string &key, T default_val) {
    T val = properties_.at(key);
    if (!val) {
      return default_val;
    }

    return val;
  }

  void AddTag(const std::string &key, BasicElementProperty element) {
    properties_.emplace(std::string(key), element);
  }

  bool RemoveTag(const std::string &key) {
    if (!HasTag(key)) {
      return false;
    }

    properties_.erase(key);
    return true;
  }

  void ClearTags() { properties_.clear(); }

  virtual std::string ToString() const {
    return std::string();
  };
};

}  // namespace osm
}  // namespace v1
}  // namespace mavix