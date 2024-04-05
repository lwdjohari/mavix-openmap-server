#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>

#if defined(MAVIX_DEBUG_CORE)
#include <iostream>
#endif

#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "mavix/v1/core/concurrent_queue.h"
#include "mavix/v1/core/round_robin_scheduler.h"
#include "mavix/v1/core/telemetry_monitor.h"
#include "mavix/v1/osm/pbf/pbf_declare.h"
#include "mavix/v1/osm/pbf/pbf_decoder.h"
#include "mavix/v1/osm/pbf/pbf_field_decoder.h"
#include "mavix/v1/osm/pbf/pbf_stream_reader.h"
#include "mavix/v1/osm/pbf/pbf_tokenizer.h"
#include "nvm/macro.h"
#include "nvm/result.h"

namespace mavix {
namespace v1 {
namespace osm {

namespace osm = mavix::v1::osm;
namespace osmf = mavix::v1::osm::formats;
namespace core = mavix::v1::core;

class OsmPbfReader {
  using Concurrent_T = core::ConcurrentQueue<pbf::PbfBlobData>;

 private:
  void (*on_reader_start_callback_)(OsmPbfReader* sender,
                                    core::StreamState state);

  void (*on_reader_finished_callback_)(OsmPbfReader* sender,
                                       core::StreamState state);

  void (*on_pbf_raw_blob_ready_)(OsmPbfReader* sender,
                                 std::shared_ptr<pbf::PbfBlobData> blob);

  void (*on_osm_data_ready_)(OsmPbfReader* sender,
                             std::shared_ptr<osm::ElementBase> osm_element);
  absl::Mutex mu_;
  absl::Mutex mu_worker_;

  absl::CondVar cv_main_worker_;
  absl::CondVar cv_all_threads_ready_;
  absl::CondVar cv_processing_;

  std::allocator<Concurrent_T> queue_shard_;
  std::thread main_worker_;
  std::vector<std::thread> process_workers_;

  uint16_t process_worker_num_;
  uint16_t max_pending_processing_;
  bool is_run_;

  core::AsyncCounter<size_t> tasks_dispatched_;
  core::AsyncCounter<size_t> tasks_finished_;
  core::AsyncCounter<size_t> tasks_received_;
  core::AsyncCounter<size_t> tasks_created_;

  core::RoundRobinScheduler round_robin_;
  std::vector<Concurrent_T*> processing_queue_ptr_;
  bool verbose_;

  pbf::PbfStreamReader stream_;
  uint16_t initialized_thread_count_;

  bool all_threads_created_;
  bool should_stop_;
  bool already_joined_;
  bool processing_already_joined_;

  Concurrent_T* CreateProcessQueue() {
    auto ptr = queue_shard_.allocate(1);
    if (!ptr) return nullptr;

    queue_shard_.construct(ptr);
    processing_queue_ptr_.push_back(ptr);

    return ptr;
  }

  bool DestroyProcessQueue(Concurrent_T* ptr) {
    if (!ptr) return false;

    queue_shard_.destroy(ptr);
    queue_shard_.deallocate(ptr, 1);

    processing_queue_ptr_.erase(std::remove(processing_queue_ptr_.begin(),
                                            processing_queue_ptr_.end(), ptr),
                                processing_queue_ptr_.end());
    return true;
  }

  void Initialize() {
    if (process_worker_num_ == 0) {
      process_worker_num_ = std::thread::hardware_concurrency();
      round_robin_.Reset(process_worker_num_);
    }
#if defined(MAVIX_DEBUG_CORE) && defined(MAVIX_DEBUG_OSM_THREAD)
    std::cout << "Hardware Thread: " << process_worker_num_ << std::endl;
#endif
    process_workers_.reserve(process_worker_num_);
  }

  void WaitForAllThreadsToBeReady() {
    absl::MutexLock lock(&mu_);
    initialized_thread_count_++;
    if (initialized_thread_count_ >= process_worker_num_ + 1) {
      all_threads_created_ = true;
      cv_all_threads_ready_.SignalAll();
    }

    if (!all_threads_created_ && is_run_ && !should_stop_) {
      cv_all_threads_ready_.Wait(&mu_);
    }
  }

  void DebugCondVar(const int16_t& worker_id, bool processing_state,
                    const std::string& process_name) {
    // #if defined(MAVIX_DEBUG_CORE) && defined(MAVIX_DEBUG_OSM_THREAD)
    //     std::cout << "Worker: " << worker_id
    //               << " {state=CV_LOCK, proc = " << process_name << "}" <<
    //               std::endl;
    // #endif
  }

  void ProcessBlockTokenizer(uint16_t worker_id,
                             uint16_t max_pending_processing,
                             std::vector<Concurrent_T*>&& queue_shard_ptr) {
    auto pq_ptr = std::forward<std::vector<Concurrent_T*>>(queue_shard_ptr);

    stream_.OnFinished(
        [this](pbf::PbfTokenizer* sender, core::StreamState state) {
          
          while (tasks_created_.Value() > tasks_finished_.Value()) {
            cv_main_worker_.WaitWithTimeout(&mu_worker_,
                                            absl::Duration(absl::Seconds(1)));

            auto c = tasks_created_.Value();
            auto f = tasks_finished_.Value();
            auto p = (static_cast<float>(f) / static_cast<float>(c)) * 100.0;
            std::cout << "Tasks " << f << "/" << c << std::fixed
                      << std::setprecision(1) << " [" << p << "%]" << std::endl;
          }

          std::cout << "PROCESSING FINISHED" << std::endl;

          absl::MutexLock lock(&mu_);
          should_stop_ = true;
          cv_processing_.SignalAll();
        });

    stream_.OnDataReady(
        [this, &pq_ptr](pbf::PbfTokenizer* sender,
                        std::shared_ptr<pbf::PbfBlobData> data) {
          tasks_created_.Inc();

          auto worker = round_robin_.Dispatch();
          // std::cout << "To Worker:" << worker << std::endl;
          auto q = pq_ptr.at(worker);

          q->Enqueue(*data);
          tasks_dispatched_.Inc();
          cv_processing_.SignalAll();
        });

    WaitForAllThreadsToBeReady();

    {
      absl::MutexLock lock(&mu_worker_);
      stream_.Start(verbose_);
    }
  }

  void ProcessOsmPbfBlob(uint16_t worker_id, Concurrent_T* queue) {
    WaitForAllThreadsToBeReady();
    {
      absl::MutexLock lock(&mu_);
      auto options = SkipOptions(stream_.DecoderOptions());

      DebugCondVar(worker_id, should_stop_, "BLOB-PROC");

      while (true) {
        if (queue->Empty() && is_run_ && !should_stop_) {
          DebugCondVar(worker_id, should_stop_, "WAITJOB");
          cv_processing_.Wait(&mu_);
        }

        if (should_stop_) {
          break;
        }

        if (queue->Empty()) {
          continue;  // Go back to waiting or ending the loop.
        }

        mu_.Unlock();

        std::shared_ptr<pbf::PbfBlobData> p =
            std::make_shared<pbf::PbfBlobData>();
        auto state = queue->TryDequeue(*p);
        tasks_received_.Inc();
        // std::cout << "Processing: " << worker_id << std::endl;

        auto decoder = std::make_shared<pbf::PbfDecoder>(p, options);
        decoder->Run();
        decoder.reset();
        p->blob_data->Destroy();
        p->blob.clear_data();

        tasks_finished_.Inc();

        mu_.Lock();
      }
    }

    std::cout << "PROCESS FINISHED: " << worker_id << std::endl;

    DebugCondVar(worker_id, should_stop_, "EXIT THREAD");
  }

  void JoinProcessWorkers() {
    if (processing_already_joined_) return;
    processing_already_joined_ = true;
    for (auto& t : process_workers_) {
      if (t.joinable()) {
        t.join();
      }
    }
  }

  void ClearProcessQueue() {
    absl::MutexLock lock(&mu_);
    // std::cout << "CLEAR-TASK" << std::endl;

    for (auto ptr : processing_queue_ptr_) {
      for (size_t i = 0; i < ptr->Size(); i++) {
        pbf::PbfBlobData p;
        ptr->TryDequeue(p);
        p.blob_data->Destroy();
      }
      ptr->Clear();
      DestroyProcessQueue(ptr);
    }
  }

 public:
  explicit OsmPbfReader(
      const std::string& filename, osm::SkipOptions options,
      uint16_t process_worker = std::thread::hardware_concurrency(),
      uint16_t max_pending_processing = 0, bool verbose = false)
      : is_run_(false),
        on_pbf_raw_blob_ready_(nullptr),
        on_reader_start_callback_(nullptr),
        on_reader_finished_callback_(nullptr),
        on_osm_data_ready_(nullptr),
        queue_shard_(),
        processing_queue_ptr_(),
        round_robin_(0),
        process_workers_(),
        process_worker_num_(process_worker),
        max_pending_processing_(max_pending_processing),
        stream_(std::string(filename), verbose),
        verbose_(verbose),
        initialized_thread_count_(0),
        all_threads_created_(false),
        should_stop_(false),
        already_joined_(false),
        tasks_dispatched_(),
        tasks_received_(),
        tasks_finished_(),
        tasks_created_(),
        processing_already_joined_(false) {
    Initialize();
  }

  ~OsmPbfReader() {}

  core::StreamState Start() {
    absl::MutexLock lock(&mu_);

    if (is_run_) return core::StreamState::Processing;

    is_run_ = true;
    should_stop_ = false;

    initialized_thread_count_ = 0;
    all_threads_created_ = false;
    already_joined_ = false;
    processing_already_joined_ = false;
    tasks_created_.Reset();
    tasks_received_.Reset();
    tasks_dispatched_.Reset();
    tasks_finished_.Reset();
    round_robin_.Reset(process_worker_num_);

    for (auto ptr : processing_queue_ptr_) {
      for (size_t i = 0; i < ptr->Size(); i++) {
        pbf::PbfBlobData p;
        ptr->TryDequeue(p);
        p.blob_data->Destroy();
      }

      ptr->Clear();
      DestroyProcessQueue(ptr);
    }

    processing_queue_ptr_.clear();

    auto state = stream_.Open();
    if (state != core::StreamState::Ok) {
      is_run_ = false;
      return state;
    }

    std::vector<Concurrent_T*> processing_ptr_for_main_worker;

    for (auto i = 0; i < process_worker_num_; i++) {
      auto qptr = CreateProcessQueue();
      processing_ptr_for_main_worker.emplace_back(qptr);
      process_workers_.emplace_back(
          std::thread(&OsmPbfReader::ProcessOsmPbfBlob, this, i + 2, qptr));
    }

    main_worker_ = std::thread(&OsmPbfReader::ProcessBlockTokenizer, this, 1,
                               max_pending_processing_,
                               std::move(processing_ptr_for_main_worker));

    return core::StreamState::Ok;
  }

  void Join() {
    if (already_joined_) return;
    already_joined_ = true;

    if (main_worker_.joinable()) {
      main_worker_.join();
    }
  }

  core::StreamState Stop() {
    // std::cout << "Stoping" << std::endl;
    {
      absl::MutexLock lock(&mu_);
      if (!is_run_) return core::StreamState::Stoped;

      if (!should_stop_) {
        should_stop_ = true;
        is_run_ = false;

        cv_processing_.SignalAll();
        cv_main_worker_.SignalAll();
      } else {
        is_run_ = false;
      }
    }
    JoinProcessWorkers();
    ClearProcessQueue();
    {
      // std::cout << "Stoped" << std::endl;

      Join();
      stream_.Stop();
    }

    return core::StreamState::Ok;
  }

  void OnFoundRawDataCallback(void (*callback)(
      OsmPbfReader* sender, std::shared_ptr<pbf::PbfBlobData> blob)) {
    absl::MutexLock lock(&mu_);
    on_pbf_raw_blob_ready_ = callback;
  }

  void OnFoundOsmDataCallback(void (*callback)(
      OsmPbfReader* sender, std::shared_ptr<osm::ElementBase> osm_element)) {
    absl::MutexLock lock(&mu_);
    on_osm_data_ready_ = callback;
  }

  void OnScanStartedCallback(void (*callback)(OsmPbfReader* sender,
                                              core::StreamState state)) {
    absl::MutexLock lock(&mu_);
    on_reader_start_callback_ = callback;
  }

  void OnScanFinishedCallback(void (*callback)(OsmPbfReader* sender,
                                               core::StreamState state)) {
    absl::MutexLock lock(&mu_);
    on_reader_finished_callback_ = callback;
  }

  void UnregisterOnFoundRawDataCallback() {
    absl::MutexLock lock(&mu_);
    on_pbf_raw_blob_ready_ = nullptr;
  }

  void UnregisterOnScanStartedCallback() {
    absl::MutexLock lock(&mu_);
    on_reader_start_callback_ = nullptr;
  }

  void UnregisterOnScanFinishedCallback() {
    absl::MutexLock lock(&mu_);
    on_reader_finished_callback_ = nullptr;
  }
};

}  // namespace osm
}  // namespace v1
}  // namespace mavix