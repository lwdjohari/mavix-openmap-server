#include <traxosm/pbf/OsmDecoder.hxx>

namespace traxosm
{
    namespace pbf
    {
        OsmDecoder::OsmDecoder()
        {
        }

        OsmDecoder::~OsmDecoder()
        {
        }

        OsmDecoderState OsmDecoder::inflateBlobContent(std::shared_ptr<nvm::StreamBuffer> rawblob, nvm::StreamBuffer *result)
        {
            if (!rawblob || !result)
                return OsmDecoderState::Nullptr;

            if (rawblob->size() == 0)
                return OsmDecoderState::SizeZero;

            auto blob = std::unique_ptr<OSMPBF::Blob>();
            auto srcPtr = static_cast<void *>(rawblob->pointerToData(0));
            auto srcSize = rawblob->size();

            if (!blob->ParseFromArray(srcPtr, srcSize))
                return OsmDecoderState::ParseBytesToProtobufFailed;

            if (blob->has_raw()) // No Compression
            {
                auto rawSize = blob->raw().size();
                const uint8_t *rawPtr = reinterpret_cast<const uint8_t *>(&blob->raw());

                if (!rawPtr)
                    return OsmDecoderState::EmptyRawData;

                result->reserve(rawSize);
                result->add(rawPtr, rawSize);

                return OsmDecoderState::Ok;
            }
            else if (blob->has_zlib_data()) // Zlib Compression
            {

                utils::ZlibCompression zc;
                bool inflateResult;

                auto uncompressedSize = blob->raw_size();
                auto rawSize = blob->zlib_data().size();
                const uint8_t *rawPtr = reinterpret_cast<const uint8_t *>(&blob->zlib_data());

                result->reserve(uncompressedSize);
                zc.inflateBytes(rawPtr, rawSize, result->bytes().get(), inflateResult);

                if (!inflateResult)
                    return OsmDecoderState::UnableToInflateCompression;

                return OsmDecoderState::Ok;
            }
            else if (blob->has_lz4_data()) // lz4 Compression
            {
                // TODO implement lz4 support on OSM raw inflate
                return OsmDecoderState::UnsupportedCompression;
            }
            else if (blob->has_lzma_data()) // lzma compression
            {
                // TODO implement lzma support on OSM raw inflate
                return OsmDecoderState::UnsupportedCompression;
            }
            else if (blob->has_zstd_data()) // zstd compression
            {
                // TODO implement zstd support on OSM raw inflate
                return OsmDecoderState::UnsupportedCompression;
            }
            else if (blob->has_obsolete_bzip2_data())
            {
                return OsmDecoderState::ObseleteFeature;
            }

            return OsmDecoderState::Error;
        }

        std::shared_ptr<formats::OSMFileheader> OsmDecoder::decodeOsmHeader(std::shared_ptr<nvm::StreamBuffer> osmdata)
        {
            auto headerBlock = std::make_unique<OSMPBF::HeaderBlock>();

            auto requiredFeatures = std::vector<std::string>();

            // Features
            // "OsmSchema-V0.6" — File contains data with the OSM v0.6 schema.
            // "DenseNodes" — File contains dense nodes and dense info.
            // "HistoricalInformation" — File contains historical OSM data.

            // We will not include HistoricalInformation features and flags it as unsupported features.
            auto supportedFeatures = std::vector<std::string>{"OsmSchema-V0.6", "DenseNodes"};
            auto unsupportedFeatures = std::vector<std::string>();

            bool isAlreadyContainedInRequiredFeatures = false;
            bool isSupportedFeatures = false;
            bool isAlreadyContainsInUnsupportedFeatures = false;
            bool isFoundOneOrMoreUnsupportedFeatures = false;

            for( auto &feature : headerBlock->required_features()){
                
                isAlreadyContainedInRequiredFeatures = nvm::vectorex::contains<std::string>(requiredFeatures, feature);
                isSupportedFeatures = nvm::vectorex::contains<std::string>(supportedFeatures, feature);
                if(!isSupportedFeatures)
                    isAlreadyContainedInRequiredFeatures = nvm::vectorex::contains<std::string>(unsupportedFeatures,feature);
                
                if(!isAlreadyContainedInRequiredFeatures){
                    requiredFeatures.emplace_back(std::string(feature));
                }

                if(!isSupportedFeatures && !isAlreadyContainsInUnsupportedFeatures){
                    unsupportedFeatures.emplace_back(std::string(feature));
                }
            }

            // If we found one or more unsupported features
            // Just return the FileHeaders with flags unsupported featurs.
            // It depends to devs to continue or resume the decoding process.
            if(unsupportedFeatures.size() > 0)
                isFoundOneOrMoreUnsupportedFeatures = true;
            

            auto osmheader = std::make_shared<formats::OSMFileheader>(!isFoundOneOrMoreUnsupportedFeatures);
            
            return std::move(osmheader);
        }
    }
}