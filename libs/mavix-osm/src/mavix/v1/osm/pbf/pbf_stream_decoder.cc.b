#include <traxosm/pbf/PbfStreamDecoder.hxx>

namespace traxosm {
namespace pbf {

PbfStreamDecoder::PbfStreamDecoder(std::shared_ptr<nvm::Stream> &&stream,
                                   size_t maxBlobs, SkipOptions skipOptions,
                                   uint16_t decoderWorker)
    : stream_(stream),
      maxPendingBlobs_(maxBlobs),
      skipOptions_(skipOptions),
      isRun_(false),
      isWorkerSpawned_(false),
      decoderWorker_(decoderWorker),
      stopSignal_(false),
      rawBlockDecoded_(0),
      rawBlockFailedToDecoded_(0),
      rawBlockFound_(0),
      rawBlockProcessed_(0) {
  pbfIterator_ = std::make_unique<PbfStreamBlobIterator>(*(stream_));
}

PbfStreamDecoder::PbfStreamDecoder(const std::string &path, size_t maxBlobs,
                                   SkipOptions skipOptions,
                                   uint16_t decoderWorker)
    : stream_(std::make_shared<nvm::Stream>(path)),
      maxPendingBlobs_(maxBlobs),
      skipOptions_(skipOptions),
      isRun_(false),
      isWorkerSpawned_(false),
      decoderWorker_(decoderWorker),
      stopSignal_(false),
      rawBlockDecoded_(0),
      rawBlockFailedToDecoded_(0),
      rawBlockFound_(0),
      rawBlockProcessed_(0)

{
  pbfIterator_ = std::make_unique<PbfStreamBlobIterator>(*(stream_));
}

PbfStreamDecoder::~PbfStreamDecoder() {
  for (auto &thread : workerThreads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

void PbfStreamDecoder::startRunable() {
  std::shared_ptr<nvm::StreamBuffer> buffer = nullptr;

  pbfIterator_->begin();          // Make sure stream cursor is on beginning.
  buffer = pbfIterator_->next();  // Get first blob.

  while (buffer != nullptr) {
    queueBlobForDecoding(buffer);  // Queue the raw blob inside StreamBuffer.
    cv_.Signal();  // Signal one of waiting thread if there available blob to
                   // decode.

    buffer = pbfIterator_->next();  // Get next blob.
  }

  isRun_ = false;
}

void PbfStreamDecoder::spawnWorkerThread() {
  if (isWorkerSpawned_) return;

  uint16_t worker = decoderWorker_ == 0 ? std::thread::hardware_concurrency()
                                        : decoderWorker_;

  decoderWorker_ = worker;

  for (uint16_t i = 0; i < worker; i++) {
    workerThreads_.emplace_back(&PbfStreamDecoder::decoderRunable, this, i);
  }

  if (workerThreads_.size() > 0) {
    isWorkerSpawned_ = true;
    decoderWorker_ = workerThreads_.size();
  }
}

void PbfStreamDecoder::queueBlobForDecoding(
    std::shared_ptr<nvm::StreamBuffer> buffer) {
  absl::MutexLock lock(&mutex_);
  queue_.push(std::move(buffer));

  // No need to lock the mutex
  // already locked inside this method
  processedCounterIncrement(DecoderProcessedFlags::RawBlockFound, false);
}

void PbfStreamDecoder::decoderRunable(uint16_t threadId) {
  // Wait for an item to be added
  // This is not endless loop because execution is suspended by absl::CondVar
  while (queue_.empty() && stopSignal_ == false) {
    cv_.Wait(&mutex_);
  }

  // Exit when stop signal received
  if (stopSignal_) return;

  std::shared_ptr<nvm::StreamBuffer> buffer = nullptr;

  // Scoping mutex lock only to pop the stream buffer from queue
  // Avoid hogging the mutex lock to long
  {
    absl::MutexLock lock(&mutex_);

    buffer = queue_.front();
    queue_.pop();
  }

  decodeNoMutexLock(buffer);
}

void PbfStreamDecoder::decodeNoMutexLock(
    std::shared_ptr<nvm::StreamBuffer> buffer) {
  if (!buffer) return;

  // Lock it because we already tell the caller that
  // this method is not going to acquired mutex lock
  processedCounterIncrement(DecoderProcessedFlags::RawBlockProcessed);

  // Posible outcome : Decoded or FailedToDecoded
  // processedCounterIncrement(DecoderProcessedFlags::RawBlockDecoded);
  // processedCounterIncrement(DecoderProcessedFlags::RawBlockFailedToDecoded);
}

void PbfStreamDecoder::processedCounterIncrement(DecoderProcessedFlags flags,
                                                 bool lockMutex) {
  if (lockMutex) absl::MutexLock lock(&mutex_);

  if ((DecoderProcessedFlags::RawBlockDecoded & flags) ==
      DecoderProcessedFlags::RawBlockDecoded) {
    rawBlockDecoded_++;
  }

  if ((flags & DecoderProcessedFlags::RawBlockFailedToDecoded) ==
      DecoderProcessedFlags::RawBlockFailedToDecoded) {
    rawBlockFailedToDecoded_++;
  }

  if ((flags & DecoderProcessedFlags::RawBlockFound) ==
      DecoderProcessedFlags::RawBlockFound) {
    rawBlockFound_++;
  }

  if ((flags & DecoderProcessedFlags::RawBlockProcessed) ==
      DecoderProcessedFlags::RawBlockProcessed) {
    rawBlockProcessed_++;
  }
}

void PbfStreamDecoder::processedCounterDecrement(DecoderProcessedFlags flags,
                                                 bool lockMutex) {
  if (lockMutex) absl::MutexLock lock(&mutex_);

  if ((flags & DecoderProcessedFlags::RawBlockDecoded) ==
      DecoderProcessedFlags::RawBlockDecoded) {
    rawBlockDecoded_--;
  }

  if ((flags & DecoderProcessedFlags::RawBlockFailedToDecoded) ==
      DecoderProcessedFlags::RawBlockFailedToDecoded) {
    rawBlockFailedToDecoded_--;
  }

  if ((flags & DecoderProcessedFlags::RawBlockFound) ==
      DecoderProcessedFlags::RawBlockFound) {
    rawBlockFound_--;
  }

  if ((flags & DecoderProcessedFlags::RawBlockProcessed) ==
      DecoderProcessedFlags::RawBlockProcessed) {
    rawBlockProcessed_--;
  }
}

void PbfStreamDecoder::processedCounterSet(size_t value,
                                           DecoderProcessedFlags flags,
                                           bool lockMutex) {
  if (lockMutex) absl::MutexLock lock(&mutex_);

  if ((flags & DecoderProcessedFlags::RawBlockDecoded) ==
      DecoderProcessedFlags::RawBlockDecoded) {
    rawBlockDecoded_ = value;
  }

  if ((flags & DecoderProcessedFlags::RawBlockFailedToDecoded) ==
      DecoderProcessedFlags::RawBlockFailedToDecoded) {
    rawBlockFailedToDecoded_ = value;
  }

  if ((flags & DecoderProcessedFlags::RawBlockFound) ==
      DecoderProcessedFlags::RawBlockFound) {
    rawBlockFound_ = value;
  }

  if ((flags & DecoderProcessedFlags::RawBlockProcessed) ==
      DecoderProcessedFlags::RawBlockProcessed) {
    rawBlockProcessed_ = value;
  }
}

std::shared_ptr<nvm::Stream> PbfStreamDecoder::stream() { return stream_; }

const uint16_t &PbfStreamDecoder::worker() const { return decoderWorker_; }

DecoderState PbfStreamDecoder::start() {
  if (!stream_) return DecoderState::StreamUnitilized;

  if (!stream_->isOpen()) return DecoderState::FileNotExist;

  if (stream_->filesize() == 0) return DecoderState::FilesizeZero;

  if (isRun_) return DecoderState::DecoderAlreadyRun;

  if (!pbfIterator_) return DecoderState::StreamUnitilized;

  isRun_ = true;
  stopSignal_ = false;

  processedCounterSet(0, DecoderProcessedFlags::All, false);

  // Spawn the decoder thread
  spawnWorkerThread();

  // check for available worker
  // resume if more than zero worker
  if (workerThreads_.size() == 0) return DecoderState::Error;

  // Spawn the producer thread
  dispatcherThread_ = std::thread(&PbfStreamDecoder::startRunable, this);

  return DecoderState::Ok;
}

void PbfStreamDecoder::wait() {
  if (!dispatcherThread_.joinable()) return;

  dispatcherThread_.join();
}

void PbfStreamDecoder::cancel() {
  absl::MutexLock lock(&mutex_);
  stopSignal_ = false;
}

bool PbfStreamDecoder::isRun() { return isRun_; }

size_t PbfStreamDecoder::blockFound() { return rawBlockFound_; }

size_t PbfStreamDecoder::blockProcessed() { return rawBlockProcessed_; }

size_t PbfStreamDecoder::blockDecoded() { return rawBlockDecoded_; }

size_t PbfStreamDecoder::blockFailedToDecoded() {
  return rawBlockFailedToDecoded_;
}
}  // namespace pbf
}  // namespace traxosm
