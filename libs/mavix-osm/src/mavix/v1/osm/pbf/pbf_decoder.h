#pragma once

#include <mavix/v1/core/core.h>

#include <stdexcept>
#include <thread>
#include <vector>

#include "absl/container/node_hash_set.h"
#include "absl/synchronization/mutex.h"
#include "mavix/v1/core/memory_buffer.h"
#include "mavix/v1/osm/element_base.h"
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
        raw_uncompressed_(nullptr),
        elements_(std::make_shared<std::vector<ElementBase>>()){
            // elements_->reserve(50);
        };

  ~PbfDecoder() {
    elements_->clear();
    if (raw_uncompressed_ &&
        compression_type_ == PbfBlobCompressionType::Zlib) {
      // std::cout << "Decoder finalized" << std::endl;
      raw_uncompressed_->Destroy();
      raw_uncompressed_ = nullptr;
    }
    data_ = nullptr;
  };

  std::shared_ptr<std::vector<ElementBase>> Elements() const {
    return elements_;
  }

  std::shared_ptr<PbfBlobData> PbfBlob() { return data_; }

  void Run() {
    elements_->clear();

    bool is_raw_available = GetBufferUncompressed();
    if (data_->header.type() == "OSMHeader" && is_raw_available) {
      // std::cout << "Process Header" << std::endl;
      ProcessOsmHeader();

    } else if (data_->header.type() == "OSMData" && is_raw_available) {
      // std::cout << "Process Data" << std::endl;
      ProcessOsmPrimitives();
    }
  }

 private:
  absl::Mutex mu_;
  std::shared_ptr<std::vector<ElementBase>> elements_;
  std::shared_ptr<PbfBlobData> data_;
  std::shared_ptr<MemoryBuffer> raw_uncompressed_;
  bool isDataValid_;
  SkipOptions skip_options_;
  PbfBlobCompressionType compression_type_;

  bool GetBufferUncompressed() {
    if (compression_type_ != PbfBlobCompressionType::None) return false;

    if (!data_->blob_data) return false;

    if (data_->blob.has_raw()) {
      compression_type_ = PbfBlobCompressionType::Raw;
      raw_uncompressed_ = data_->blob_data;

      return true;

    } else if (data_->blob.has_zlib_data()) {
      compression_type_ = PbfBlobCompressionType::Zlib;

      auto zlib = GetCompression<ZlibCompression>();
      bool state;

      raw_uncompressed_ = zlib.Inflate(data_->blob_data->Data(),
                                       data_->blob_data->Size(), state);

      return state;
    } else {
      // Not yet supported
      throw std::runtime_error("Compresion not supported");
    }
  }

  void ProcessOsmHeader() {
    auto pbf_header = OSMPBF::HeaderBlock();
    auto is_parsed = pbf_header.ParseFromArray(raw_uncompressed_->Data(),
                                               raw_uncompressed_->Size());
    //     if (!is_parsed) {
    // #if defined(MAVIX_DEBUG_CORE) && defined(MAVIX_DEBUG_PBF_DECODER)
    //       std::cerr << "OsmHeader: parsed from bytes failed" << std::endl;
    // #endif
    //       return;
    //     }

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

#if defined(MAVIX_DEBUG_CORE) && defined(MAVIX_DEBUG_PBF_DECODER)
    std::cout << header_file.ToString() << std::endl;
#endif
    formats::HeaderBBox bound;
    if (pbf_header.has_bbox()) {
      auto pbf_bbox = pbf_header.bbox();
      bound = formats::HeaderBBox(
          pbf_bbox.right() * ElementBase::COORDINATE_SCALING_FACTOR,
          pbf_bbox.left() * ElementBase::COORDINATE_SCALING_FACTOR,
          pbf_bbox.top() * ElementBase::COORDINATE_SCALING_FACTOR,
          pbf_bbox.bottom() * ElementBase::COORDINATE_SCALING_FACTOR,
          pbf_header.source());

    } else {
      bound = formats::HeaderBBox(pbf_header.source());
    }

    elements_->emplace_back(std::move(header_file));
    // elements_->emplace_back(std::move(bound));
  }

  void ProcessOsmPrimitives() {
    auto primitive_block = OSMPBF::PrimitiveBlock();
    auto is_parsed = primitive_block.ParseFromArray(raw_uncompressed_->Data(),
                                                    raw_uncompressed_->Size());
    if (!is_parsed) {
#if defined(MAVIX_DEBUG_CORE) && defined(MAVIX_DEBUG_PBF_DECODER)
      std::cerr << "Failed parsing PRIMITIVE GROUPS from bytes stream"
                << std::endl;
#endif
      return;
    }

    auto pbf_field_decoder = PbfFieldDecoder(primitive_block);

    auto is_skip_nodes =
        (skip_options_ & SkipOptions::Nodes) == SkipOptions::Nodes;
    auto is_skip_ways =
        (skip_options_ & SkipOptions::Ways) == SkipOptions::Ways;
    auto is_skip_relations =
        (skip_options_ & SkipOptions::Relations) == SkipOptions::Relations;

    if (skip_options_ == SkipOptions::None) {
      is_skip_nodes = false;
      is_skip_relations = false;
      is_skip_ways = false;
    }

    for (auto &pg : primitive_block.primitivegroup()) {
      if (!is_skip_nodes) {
        // !TODO: Implement overload method ProcessNodes for Dense Nodes

        ProcessNodes(pg.dense(), pbf_field_decoder);
        ProcessNodes(pg.nodes(), pbf_field_decoder);
      }

      if (!is_skip_ways) {
        ProcessWays(pg.ways(), pbf_field_decoder);
      }

      // if (!is_skip_relations) {
      //   ProcessRelations(pg.relations(), pbf_field_decoder);
      // }
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
    if (nodes.size() == 0) return;
    std::cout << "Nodes count: " << nodes.size() << std::endl;
    for (auto &node : nodes) {
      auto tags = ComposeTags(node.keys(), node.vals(), field_decoder);

      auto osm_node = formats::Node(
          node.id(), field_decoder.DecodeLatitude(node.lat()),
          field_decoder.DecodeLongitude(node.lon()), std::move(tags.unwrap()));

#if defined(MAVIX_DEBUG_CORE) && defined(MAVIX_DEBUG_PBF_DECODER)
      std::cout << osm_node.ToString() << std::endl;
#endif
      elements_->emplace_back(std::move(osm_node));
    }
  }

  void ProcessNodes(const OSMPBF::DenseNodes &node,
                    PbfFieldDecoder field_decoder) {
    if (node.id_size() == 0) return;
    std::cout << "Dense Nodes count: " << node.id_size() << std::endl;

    auto ids = node.id();
    auto lats = node.lat();
    auto lons = node.lon();

    if (node.id_size() != node.lon_size() ||
        node.id_size() != node.lat_size()) {
      return;
    }

    auto kv_iterator = node.keys_vals().begin();

    int64_t node_id = 0;
    int64_t lat = 0;
    int64_t lon = 0;

    for (size_t i = 0; i < node.id_size(); i++) {
      node_id += ids.at(i);
      lat += lats.at(i);
      lon += lons.at(i);

      absl::node_hash_map<std::string, BasicElementProperty> tags;
      while (kv_iterator != node.keys_vals().end()) {
        auto key_index = *kv_iterator;
        if (key_index == 0) {
          break;
        }

        kv_iterator++;
        auto value_index = *kv_iterator;

        kv_iterator++;
      }

      auto osm_node =
          formats::Node(node_id, field_decoder.DecodeLatitude(lat),
                        field_decoder.DecodeLongitude(lon), std::move(tags));

      elements_->emplace_back(std::move(osm_node));
    }

    // Map<String, Object> tags = null;
    // while (keysValuesIterator.hasNext()) {
    //   int keyIndex = keysValuesIterator.next();
    //   if (keyIndex == 0) {
    //     break;
    //   }
    //   if (checkData) {
    //     if (!keysValuesIterator.hasNext()) {
    //       throw new RuntimeException(
    //           "The PBF DenseInfo keys/values list contains a key with no "
    //           "corresponding value.");
    //     }
    //   }
    //   int valueIndex = keysValuesIterator.next();

    //   if (tags == null) {
    //     // divide by 2 as key&value, multiple by 2 because of the better
    //     // approximation
    //     tags = new HashMap<>(Math.max(
    //         3, 2 * (nodes.getKeysValsList().size() / 2) / idList.size()));
    //   }

    //   tags.put(fieldDecoder.decodeString(keyIndex),
    //            fieldDecoder.decodeString(valueIndex));
    // }

    // ReaderNode node =
    //     new ReaderNode(nodeId, fieldDecoder.decodeLatitude(latitude),
    //                    fieldDecoder.decodeLongitude(longitude));
    // node.setTags(tags);

    // // Add the bound object to the results.
    // decodedEntities.add(node);
    //}
  }

  void ProcessWays(const protobuf::RepeatedPtrField<OSMPBF::Way> &ways,
                   const PbfFieldDecoder &field_decoder) {
    if (ways.size() == 0) return;
    std::cout << "Ways count: " << ways.size() << std::endl;
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

#if defined(MAVIX_DEBUG_CORE) && defined(MAVIX_DEBUG_PBF_DECODER)
      // std::cout << osm_way.ToString() << std::endl;
#endif

      elements_->emplace_back(std::move(osm_way));
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

      elements_->emplace_back(std::move(relation));
      member_id_iterator++;
      member_role_iterator++;
      member_type_iterator++;
    }

    return true;
  }

  void ProcessRelations(
      const protobuf::RepeatedPtrField<OSMPBF::Relation> &relations,
      const PbfFieldDecoder &field_decoder) {
        if (relations.size() == 0) return;
    std::cout << "Relations count: " << relations.size() << std::endl;

    for (auto &relation : relations) {
      auto tags = ComposeTags(relation.keys(), relation.vals(), field_decoder);

      // std::cout << "Rel Tags:" << tags.unwrap().size() << std::endl;
      auto osm_relation =
          formats::Relation(relation.id(), std::move(tags.unwrap()));

      ComposeRelationMembers(osm_relation, relation.memids(),
                             relation.roles_sid(), relation.types(),
                             field_decoder);

#if defined(MAVIX_DEBUG_CORE) && defined(MAVIX_DEBUG_PBF_DECODER)
      // std::cout << osm_relation.ToString() << std::endl;
#endif
      elements_->emplace_back(std::move(osm_relation));
    }
  }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix
