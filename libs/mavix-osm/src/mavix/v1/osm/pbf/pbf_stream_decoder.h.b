#pragma once

#include <mavix/v1/core/core.h>
#include <absl/synchronization/mutex.h>
#include <absl/synchronization/notification.h>
#include <vector>
#include <queue>
#include <thread>
#include <exception>
#include <memory>

#include "nvm/macro.h"
#include "mavix/v1/osm/pbf/pbf_stream_blob_iterator.h"
#include "mavix/v1/osm/skip_options.h"
#include "zlib.h"

namespace traxosm
{
    namespace pbf
    {
        enum class DecoderState
        {
            Error = 0,
            Ok = 1,
            FileNotExist = 2,
            MismatchRawBlobSize = 3,
            StreamUnitilized = 4,
            FilesizeZero = 5,
            DecoderAlreadyRun = 6
        };

        enum class DecoderProcessedFlags:uint8_t{
            None = 0,
            RawBlockFound = 1,
            RawBlockProcessed = 2,
            RawBlockFailedToDecoded = 4,
            RawBlockDecoded = 8,
            All = 15
        };

        NVM_ENUMCLASS_ENABLE_BITMASK_OPERATORS(DecoderProcessedFlags)

        /// @brief  Class for decoding PBF file into OSM Data Structures.
        ///         This class designed for multi-threaded processing by utilized
        ///         one-producer and multi-consumer approach.
        ///         Use wait() method for waiting until decoder finished all processing.
        /// @author Linggawasistha Djohari <linggawasistha.djohari@outlook.com>
        class PbfStreamDecoder
        {
        private:
            
            
            std::unique_ptr<PbfStreamBlobIterator> pbfIterator_;   // OSM PBF Stream Iterator.
            std::shared_ptr<nvm::Stream> stream_;                  // Underlying to OSM PBF filestream.
            std::queue<std::shared_ptr<nvm::StreamBuffer>> queue_; // Queue of stream buffer waiting for decoding process.

            bool isRun_;             // State for decoding process run or not.
            bool isWorkerSpawned_;   // State to flag if decoders/consumers thread already spawned
            bool stopSignal_;        // State to flag producer and consumers thread to leave the executions
            size_t maxPendingBlobs_; // Number of max thread before can be queue, 0 (zero) will be no-max.

            size_t rawBlockFound_;  // Numbers of raw blob transfered to consumer thread for decoding
            size_t rawBlockDecoded_; // Numbers of successful decoding.
            size_t rawBlockFailedToDecoded_; // Numbers of failed decoding.
            size_t rawBlockProcessed_;  // Total numbers processed (sucessful decoding + failed decoding).

            uint16_t decoderWorker_; // Number of consumer thread.

            absl::Mutex mutex_;    // Mutex for locking.
            absl::CondVar cv_;     // CV for signaling consumers thread.
            absl::CondVar cvWait_; // CV for suspend main thread to wait until all decoding process finished.

            SkipOptions skipOptions_; // Flags to skip certain OSM Object.

            std::thread dispatcherThread_;           // Producer Thread.
            std::vector<std::thread> workerThreads_; // Consumer Thread.

            void (*eofCallback_)(size_t datablockFound);
            void (*OsmBlobReadyCallback_)(std::shared_ptr<nvm::StreamBuffer>, std::string type, size_t datablockNum);
            void (*OsmFormatReadyCallback_)(std::shared_ptr<nvm::StreamBuffer>, std::string type, size_t datablockNum);

            void startRunable();
            void spawnWorkerThread();
            void decoderRunable(uint16_t threadId);
            void queueBlobForDecoding(std::shared_ptr<nvm::StreamBuffer> buffer);
            void decodeNoMutexLock(std::shared_ptr<nvm::StreamBuffer> buffer);

            void processedCounterIncrement(DecoderProcessedFlags flags = DecoderProcessedFlags::None, bool lockMutex = true);
            void processedCounterDecrement(DecoderProcessedFlags flags = DecoderProcessedFlags::None, bool lockMutex = true);
            void processedCounterSet(size_t value, DecoderProcessedFlags flags = DecoderProcessedFlags::None, bool lockMutex = true);


        public:
            explicit PbfStreamDecoder(
                std::shared_ptr<nvm::Stream> &&stream, 
                size_t maxBlobs, SkipOptions skipOptions = SkipOptions::None, 
                uint16_t decoderWorker = std::thread::hardware_concurrency());

            explicit PbfStreamDecoder(
                const std::string &path, size_t maxBlobs, 
                SkipOptions skipOptions = SkipOptions::None, 
                uint16_t decoderWorker = std::thread::hardware_concurrency());

            ~PbfStreamDecoder();

            /// @brief Get underlying stream
            /// @return nvm::Stream to the underlying PBF file.
            std::shared_ptr<nvm::Stream> stream();

            /// @brief Return number of decoder thread.
            ///         Param with zero value will use all available hardware thread,
            ///         Minimum worker is one worker thread.
            /// @return Number of thread use by decoder for processing the OSM PBF Blob.
            const uint16_t &worker() const;

            /// @brief Start multi-thread OSM PBF decoding process.
            /// @return DecoderState::Ok if process started successfuly.
            DecoderState start();

            /// @brief Caller thread will wait until decoding process finished.
            void wait();

            /// @brief Cancel decoder processing.
            void cancel();

            /// @brief Get Decoder is processing data or not.
            /// @return true if run
            bool isRun();

            /// @brief Raw OSM PBF Blob found from stream.
            ///         This number will increase along with decoding process.
            /// @return Number of blocb/block found so far.
            size_t blockFound();

            /// @brief  Raw OSM blob being decoded.
            ///         This number will increase along with decoding process.
            /// @return 
            size_t blockProcessed();

            /// @brief  Numbers of OSM Blob successfuly decoded.
            ///         This number will increase along with decoding process.
            /// @return 
            size_t blockDecoded();

            /// @brief  Numbers of OSM Blob failed to decoded.
            ///         This number will increase along with decoding process.
            /// @return 
            size_t blockFailedToDecoded();
        };

    }
}
#endif