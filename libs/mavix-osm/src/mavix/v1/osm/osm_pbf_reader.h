#pragma once

#include <mavix/v1/core/core.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#if defined(MAVIX_DEBUG_CORE)
#include <iostream>
#endif

#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "mavix/v1/core/async_counter.h"
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

  absl::CondVar cv_main_worker_;
  absl::CondVar cv_all_threads_ready_;
  absl::CondVar cv_processing_;

  std::queue<std::shared_ptr<pbf::PbfBlobData>> process_queue_;
  std::thread main_worker_;
  std::vector<std::thread> process_workers_;

  uint16_t process_worker_num_;
  uint16_t max_pending_processing_;
  bool is_run_;

  core::AsyncCounter<size_t> tasks_dispatched_;
  core::AsyncCounter<size_t> tasks_finished_;
  core::AsyncCounter<size_t> tasks_received_;
  core::AsyncCounter<size_t> tasks_created_;

  bool verbose_;

  pbf::PbfStreamReader stream_;
  uint16_t initialized_thread_count_;

  bool all_threads_created_;
  bool should_stop_;
  bool already_joined_;
  bool processing_already_joined_;


  void Initialize() {
    if (process_worker_num_ == 0) {
      process_worker_num_ = std::thread::hardware_concurrency();
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
//               << " {state=CV_LOCK, proc = " << process_name << "}" << std::endl;
// #endif
  }

  void ProcessBlockTokenizer(uint16_t worker_id,
                                uint16_t max_pending_processing) {
    WaitForAllThreadsToBeReady();

    {
      absl::MutexLock lock(&mu_);
      
      
      stream_.OnDataReady([this](pbf::PbfTokenizer* sender,
                                 std::shared_ptr<pbf::PbfBlobData> data) {
        
        this->tasks_created_.Inc();
        this->process_queue_.emplace(std::move(data));
        this->tasks_dispatched_.Inc();
        cv_processing_.Signal();
      });

   
      stream_.Start(verbose_);

      stream_.OnDataReadyUnregister();

      if (tasks_created_.Value() == tasks_dispatched_.Value()) {
        while (tasks_created_.Value() > tasks_finished_.Value()) {
          DebugCondVar(worker_id, true, "WAIT-PROC-FINISHED");
          cv_main_worker_.Wait(&mu_);
        }
      }
    }

    DebugCondVar(worker_id, true, "PROC-FINISHED");
  }

  void CheckForTaskFinished() {
      cv_main_worker_.SignalAll();
    if (tasks_dispatched_.Value() == tasks_finished_.Value()) {
      DebugCondVar(0, true, "TASk DONE");
      should_stop_ = true;
      cv_processing_.SignalAll();
    }
  }

  void ProcessOsmPbfBlob(uint16_t worker_id) {
     WaitForAllThreadsToBeReady(); 
    {
      absl::MutexLock lock(&mu_);
      DebugCondVar(worker_id, should_stop_, "BLOB-PROC");

      while (true) {
        if (process_queue_.empty() && is_run_ && !should_stop_) {
          DebugCondVar(worker_id, should_stop_, "WAITJOB");
          cv_processing_.Wait(&mu_);
        }

        if (should_stop_) {
          DebugCondVar(worker_id, should_stop_, "END-PROC");
          break;
        }

         if (process_queue_.empty()) {
            continue; // Go back to waiting or ending the loop.
        }

        auto options = SkipOptions(stream_.DecoderOptions());
        auto p = process_queue_.front();
        process_queue_.pop();
        // std::cout << "Worker["<< worker_id <<"]:" <<  p->header.type() << std::endl;

        mu_.Unlock();
        tasks_received_.Inc();

        auto decoder = std::make_shared<pbf::PbfDecoder>(p,options);
        decoder->Run();
        decoder.reset();
        p->blob_data->Destroy();
        tasks_finished_.Inc();
        mu_.Lock();
        CheckForTaskFinished();
      }
    }

    DebugCondVar(worker_id, should_stop_, "EXIT THREAD");
  }

  void JoinProcessWorkers() {
    if(processing_already_joined_)
    return;
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

    for (size_t i = 0; i < process_queue_.size(); i++) {
      auto p = process_queue_.front();
      p->blob_data->Destroy();
      process_queue_.pop();

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
        process_queue_(),
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

    auto state = stream_.Open();
    if (state != core::StreamState::Ok) {
      is_run_ = false;
      return state;
    }

    for (auto i = 0; i < process_worker_num_; i++) {
      process_workers_.emplace_back(
          std::thread(&OsmPbfReader::ProcessOsmPbfBlob, this, i + 2));
    }

    main_worker_ = std::thread(&OsmPbfReader::ProcessBlockTokenizer, this, 1,
                               max_pending_processing_);

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
      }else{
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