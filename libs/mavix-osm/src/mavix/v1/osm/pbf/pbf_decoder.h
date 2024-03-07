#pragma once

#include <mavix/v1/core/core.h>

#include <thread>
#include <vector>
#include <stdexcept>
#include "absl/container/node_hash_set.h"
#include "absl/synchronization/mutex.h"
#include "mavix/v1/core/memory_buffer.h"
#include "mavix/v1/osm/formats/header_bbox.h"
#include "mavix/v1/osm/formats/node.h"
#include "mavix/v1/osm/formats/osm_file_header.h"
#include "mavix/v1/osm/formats/relation.h"
#include "mavix/v1/osm/formats/way.h"
#include "mavix/v1/osm/pbf/pbf_declare.h"
#include "mavix/v1/osm/pbf/pbf_field_decoder.h"
#include "mavix/v1/osm/skip_options.h"
#include "mavix/v1/utils/compression.h"
#include "osmpbf/osmpbf.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace pbf {

using namespace mavix::v1::core;
using namespace mavix::v1::osm;
using namespace mavix::v1::utils;
namespace protobuf = google::protobuf;

enum class PbfBlobCompressionType {
  None = 0,
  Raw = 1,
  Zlib = 3,
  Lzma = 4,
  bzip2 = 5,
  Lz4 = 6,
  ZStd = 7
};

class PbfDecoder {
 public:
  explicit PbfDecoder(std::shared_ptr<PbfBlobData> data, SkipOptions options)
      : mu_(),
        data_(data),
        isDataValid_(false),
        skip_options_(options),
        compression_type_(PbfBlobCompressionType::None),
        raw_uncompressed_(nullptr){

        };

  ~PbfDecoder() {
    if (raw_uncompressed_ &&
        compression_type_ == PbfBlobCompressionType::Zlib) {
      raw_uncompressed_->Destroy();
    }
  };

  std::shared_ptr<PbfBlobData> PbfBlob() { return data_; }

  void Decode() {
    if (compression_type_ != PbfBlobCompressionType::None) return;

    if (!data_->blob_data) return;

    if (data_->blob.has_raw()) {
      compression_type_ = PbfBlobCompressionType::Raw;
      raw_uncompressed_ = data_->blob_data;

    } else if (data_->blob.has_zlib_data()) {
      compression_type_ = PbfBlobCompressionType::Zlib;

      auto zlib = GetCompression<ZlibCompression>();
      bool state;

      raw_uncompressed_ = zlib.Inflate(data_->blob_data->Data(),
                                       data_->blob_data->Size(), state);
    }else{
      // Not yet supported
      throw std::runtime_error("Compresion not supported");
    }
  }

 private:
  absl::Mutex mu_;

  std::shared_ptr<PbfBlobData> data_;
  std::shared_ptr<MemoryBuffer> raw_uncompressed_;
  bool isDataValid_;
  SkipOptions skip_options_;
  PbfBlobCompressionType compression_type_;

  void DecodeOsmHeader() {
    auto pbf_header = OSMPBF::HeaderBlock();
    auto is_parsed = pbf_header.ParseFromArray(raw_uncompressed_->Data(),
                                               raw_uncompressed_->Size());
    if (!is_parsed) {
      return;
    }

    absl::node_hash_set<std::string> features_supported = {"OsmSchema-V0.6",
                                                           "DenseNodes"};
    absl::node_hash_set<std::string> features_unsupported;

    for (auto &f : pbf_header.required_features()) {
      if (!features_supported.contains(f)) {
        if (!features_unsupported.contains(f)) {
          features_supported.emplace(std::string(f));
        }
      }
    }

    auto header_file = formats::OSMFileheader();
    auto timestamp = pbf_header.osmosis_replication_timestamp();
    header_file.AddTag(
        "timestamp",
        ElementProperty(timestamp, KnownPropertyType::Int64, std::string()));

    if (pbf_header.has_bbox()) {
      auto pbf_bbox = pbf_header.bbox();
      auto bound = formats::HeaderBBox(
          pbf_bbox.right() * ElementBase::COORDINATE_SCALING_FACTOR,
          pbf_bbox.left() * ElementBase::COORDINATE_SCALING_FACTOR,
          pbf_bbox.top() * ElementBase::COORDINATE_SCALING_FACTOR,
          pbf_bbox.bottom() * ElementBase::COORDINATE_SCALING_FACTOR,
          pbf_header.source());

    } else {
      auto bound = formats::HeaderBBox(pbf_header.source());
    }
  }

  void DecodeOsmPrimitives() {
    auto primitive_block = OSMPBF::PrimitiveBlock();
    auto is_parsed = primitive_block.ParseFromArray(raw_uncompressed_->Data(),
                                                    raw_uncompressed_->Size());
    if (!is_parsed) {
      return;
    }

    auto pbf_field_decoder = PbfFieldDecoder(primitive_block);

    auto is_skip_nodes =
        (skip_options_ & SkipOptions::Nodes) == SkipOptions::Nodes;
    auto is_skip_ways =
        (skip_options_ & SkipOptions::Ways) == SkipOptions::Ways;
    auto is_skip_relations =
        (skip_options_ & SkipOptions::Relations) == SkipOptions::Relations;

    for (auto &pg : primitive_block.primitivegroup()) {
      if (!is_skip_nodes) {
        // !TODO: Implement overload method ProcessNodes for Dense Nodes
        // ProcessNodes(pg.dense(), pbf_field_decoder);
        ProcessNodes(pg.nodes(), pbf_field_decoder);
      }

      if (!is_skip_ways) {
        ProcessWays(pg.ways(), pbf_field_decoder);
      }

      if (!is_skip_relations) {
        ProcessRelations(pg.relations(), pbf_field_decoder);
      }
    }
  }

  Option<absl::node_hash_map<std::string, BasicElementProperty>> ComposeTags(
      const protobuf::RepeatedField<uint32_t> &keys,
      const protobuf::RepeatedField<uint32_t> &values,
      const PbfFieldDecoder &field_decoder) {
    if (keys.size() != values.size()) {
      return Option<absl::node_hash_map<std::string, BasicElementProperty>>();
    }

    auto key_iterator = keys.begin();
    auto value_iterator = values.begin();

    absl::node_hash_map<std::string, BasicElementProperty> tags;

    while (key_iterator != keys.end() && value_iterator != values.end()) {
      auto key = field_decoder.GetString(*key_iterator).unwrap();
      auto value = field_decoder.GetString(*value_iterator).unwrap();
      tags.emplace(
          std::move(key),
          ElementProperty<std::string>(
              std::move(value), KnownPropertyType::String, std::string()));

      key_iterator++;
      value_iterator++;
    }

    return Option<absl::node_hash_map<std::string, BasicElementProperty>>(
        std::move(tags));
  }

  void ProcessNodes(const protobuf::RepeatedPtrField<OSMPBF::Node> &nodes,
                    const PbfFieldDecoder &field_decoder) {
    for (auto &node : nodes) {
      auto tags = ComposeTags(node.keys(), node.vals(), field_decoder);

      auto osm_node = formats::Node(
          node.id(), field_decoder.DecodeLatitude(node.lat()),
          field_decoder.DecodeLongitude(node.lon()), std::move(tags.unwrap()));

     
      // elements_.emplace_back(osm_node);
    }
  }

  void ProcessWays(const protobuf::RepeatedPtrField<OSMPBF::Way> &ways,
                   const PbfFieldDecoder &field_decoder) {
    for (auto &way : ways) {
      auto tags = ComposeTags(way.keys(), way.vals(), field_decoder);

      auto osm_way = formats::Way(way.id(), std::move(tags.unwrap()));
      osm_way.InitializeNodes(way.refs_size());

      // The node ids are delta encoded.
      // id is stored as a delta against
      // the previous one.

      int64_t node_id = 0;
      for (auto &node_offset_id : way.refs()) {
        node_id += node_offset_id;
        osm_way.Nodes().emplace_back(node_id);
      }

      // elements_.emplace_back(osm_way);
    }
  }

  bool ComposeRelationMembers(
      formats::Relation &relation,
      const protobuf::RepeatedField<int64_t> &member_ids,
      const protobuf::RepeatedField<int32_t> &member_roles,
      const protobuf::RepeatedField<int32_t> &member_types,
      const PbfFieldDecoder &field_decoder) {
    if ((member_ids.size() != member_roles.size()) ||
        (member_ids.size() != member_types.size())) {
      return false;
    }

    auto member_id_iterator = member_ids.begin();
    auto member_role_iterator = member_roles.begin();
    auto member_type_iterator = member_types.begin();

    // The member ids are delta encoded.
    // id is stored as a delta against
    // the previous one.
    int64_t ref_id = 0;
    while (member_id_iterator != member_ids.end() &&
           member_role_iterator != member_roles.end() &&
           member_type_iterator != member_types.end()) {
      auto member_id = *member_id_iterator;
      auto member_role = *member_role_iterator;
      auto member_type = *member_type_iterator;

      ref_id += member_id;
      ElementType type = ElementType::Unknown;
      if (member_type == OSMPBF::Relation_MemberType::Relation_MemberType_WAY) {
        type = ElementType::Way;
      } else if (member_type ==
                 OSMPBF::Relation_MemberType::Relation_MemberType_RELATION) {
        type = ElementType::Relation;
      } else if (member_type ==
                 OSMPBF::Relation_MemberType::Relation_MemberType_NODE) {
        type = ElementType::Node;
      }

      if (type == ElementType::Unknown) {
        return false;
      }

      auto member = formats::RelationMember(
          type, ref_id, field_decoder.GetString(member_role).unwrap());

      relation.Add(std::move(member));
    }

    return true;
  }

  void ProcessRelations(
      const protobuf::RepeatedPtrField<OSMPBF::Relation> &relations,
      const PbfFieldDecoder &field_decoder) {
    for (auto &relation : relations) {
      auto tags = ComposeTags(relation.keys(), relation.vals(), field_decoder);

      auto osm_relation =
          formats::Relation(relation.id(), std::move(tags.unwrap()));

      ComposeRelationMembers(osm_relation, relation.memids(),
                           relation.roles_sid(), relation.types(),
                           field_decoder);

      // elements_.emplace_back(osm_relation);
    }
  }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix
