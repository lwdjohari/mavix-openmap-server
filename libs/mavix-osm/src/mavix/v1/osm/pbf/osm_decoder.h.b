#ifndef TRAXOSM_PBF_V1_OSM_DECODER_H
#define TRAXOSM_PBF_V1_OSM_DECODER_H

#include <cstdint>
#include <memory>
#include <osmpbf/osmpbf.h>
#include <nvm/VectorEx.hxx>
#include <nvm/StreamBuffer.hxx>
#include <traxosm/formats/OSMFileheader.hxx>
#include <traxosm/utilities/ZlibCompression.hxx>

namespace formats = traxosm::formats;
namespace utils = traxosm::utilities;

namespace traxosm
{
    namespace pbf
    {
        enum class OsmDecoderState:uint8_t{
            Error = 0,
            Ok = 1,
            Nullptr = 2,
            UnsupportedCompression = 3,
            UnableToInflateCompression = 4,
            UnableToProcessPBFBlob = 5,
            SkippingUnrecognicedBlobType = 6,
            MemberTypeIsNotSupported = 7,
            PBFDenseInfoKVContainsNoValue = 8,
            UnsupportedFeatures = 9,
            SizeZero = 10,
            ParseBytesToProtobufFailed = 11,
            EmptyRawData = 12,
            ObseleteFeature = 13
        };

        class OsmDecoder
        {
        private:
            
        public:
            OsmDecoder();
            ~OsmDecoder();

            OsmDecoderState inflateBlobContent(std::shared_ptr<nvm::StreamBuffer> rawblob,nvm::StreamBuffer *result);
            std::shared_ptr<formats::OSMFileheader> decodeOsmHeader(std::shared_ptr<nvm::StreamBuffer> osmdata);
            void buildTags();
            void buildRelationMembers();
            void decodeNodes();
            void decodeWays();
            void decodeRelations();
            void decodeOsmPrimitives(std::shared_ptr<nvm::StreamBuffer> buffer);

        };
        

    }
}
#endif