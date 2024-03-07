#pragma once

#include <mavix/v1/core/core.h>

#include <thread>
#include <vector>

#include "absl/container/node_hash_set.h"
#include "absl/synchronization/mutex.h"
#include "mavix/v1/core/memory_buffer.h"
#include "mavix/v1/osm/formats/header_bbox.h"
#include "mavix/v1/osm/formats/osm_file_header.h"
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
        (skip_options_ & SkipOptions::Nodes) == SkipOptions::Ways;
    auto is_skip_relations =
        (skip_options_ & SkipOptions::Relations) == SkipOptions::Ways;

    for (auto &pg : primitive_block.primitivegroup()) {
      if (!is_skip_nodes) {
        // ProcessNodes(pg.dense(), pbf_field_decoder);
        // ProcessNodes(pg.nodes(), pbf_field_decoder);
      }

      if (!is_skip_ways) {
        // ProcessWays(pg.ways(), pbf_field_decoder);
      }

      if (!is_skip_relations) {
        // ProcessRelations(pg.relations(), pbf_field_decoder);
      }
    }
  }

  void processNodes(const google::protobuf::RepeatedField<OSMPBF::Node> &nodes,
                    const PbfFieldDecoder &field_decoder) {
    for (auto &node : nodes) {
      // absl::node_hash_map<std::string, BasicElementProperty> tags =
      //     BuildTags(node.keys(), node.vals(), field_decoder);

      // ReaderNode osmNode = new ReaderNode(
      //     node.getId(), fieldDecoder.decodeLatitude(node.getLat()),
      //     fieldDecoder.decodeLatitude(node.getLon()));
      // osmNode.setTags(tags);

      // // Add the bound object to the results.
      // decodedEntities.add(osmNode);
    }
  }
};

}  // namespace pbf
}  // namespace osm
}  // namespace v1
}  // namespace mavix
