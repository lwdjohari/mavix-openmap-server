#include <mavix/v1/core/core.h>

#include <algorithm>
#include <cstdint>
#include <sstream>

#include "absl/container/node_hash_map.h"
#include "mavix/v1/osm/element_base.h"
#include "nvm/option.h"
namespace mavix {
namespace v1 {
namespace osm {
namespace formats {

using namespace nvm;

class RelationMember {
 private:
  ElementType type_;
  int64_t ref_;
  std::string role_;

 public:
  RelationMember()
      : type_(ElementType::Unknown), ref_(0), role_(std::string()) {}

  explicit RelationMember(ElementType type, int64_t ref,
                          const std::string& role)
      : type_(type), ref_(ref), role_(std::string(role)) {}

  ~RelationMember(){};

  ElementType Type() const { return type_; }

  int64_t Ref() const { return ref_; }

  std::string Role() const { return role_; }
};

class Relation : ElementBase {
 private:
  std::vector<RelationMember> members_;

 public:
  explicit Relation(int64_t id)
      : ElementBase(id, ElementType::Relation,
                    absl::node_hash_map<std::string, BasicElementProperty>()),
        members_(){};

  ~Relation(){};

  void Add(RelationMember&& member) {
    members_.emplace_back(std::forward<RelationMember>(member));
  }

  std::vector<RelationMember>& Members() { return members_; }

  Option<RelationMember&> Member(const size_t& index) {
    if (index >= members_.size()) return Option<RelationMember&>();

    return Option<RelationMember&>(members_.at(index));
  }

  bool Remove(const size_t& index) {
    if (index >= members_.size()) return false;

    members_.erase(members_.begin() + index);
    return true;
  }

  void Clear() {
    if (members_.size() == 0) return;

    members_.clear();
  }

  bool IsMetaRelation() {
    if (members_.size() == 0) return false;
    auto it = std::find_if(members_.begin(), members_.end(),
                           [](const RelationMember& member) {
                             return member.Type() == ElementType::Relation;
                           });
    return it != members_.end();
  }
};

}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix