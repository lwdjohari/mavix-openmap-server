#include "mavix/v1/osm/formats/osm_file_header.h"

namespace mavix {
namespace v1 {
namespace osm {
namespace formats {

OSMFileheader::OSMFileheader(bool isSupportedFeature)
    : isSupportedFeatures_(isSupportedFeature) {}

OSMFileheader::~OSMFileheader() {}

bool OSMFileheader::isSupportedFeatures() { return isSupportedFeatures_; }

}  // namespace formats
}  // namespace osm
}  // namespace v1
}  // namespace mavix