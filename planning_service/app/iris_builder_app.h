/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file iris_builder_app.h

#pragma once

#include "planning_service/comms/iris_service.h"
#include "planning_service/comms/registry_service.h"
#include "planning_service/comms/server_wrapper.h"

using service::iris::IrisBuildManager;
class IrisGenerationApp {
 public:
  /**
   * @brief Construct a new IRIS Generation App
   *
   * @param addr Address at which the IRIS builder will listen for new
   * requests
   * @param system_name Enabled system name
   * @param max_queue_size Maximum number of plans which may be in the queue at
   * @param max_active_plans Maximum number of plans which may be actively
   * computed simultaneously
   */
  IrisGenerationApp(const std::string& addr, const std ::string& system_name,
                    const service::utils::ResourceOptions& options,
                    const size_t max_queue_size = 10, const size_t max_jobs = 1)
      : addr_ {addr} {
    mgr_ = std::make_shared<IrisBuildManager>(system_name, options,
                                              max_queue_size, max_jobs);
    srv_ = std::make_unique<comms::ServerWrapper>(addr_);
    srv_->AddService(std::make_unique<comms::IrisBuilderService>(mgr_));
    srv_->AddService(
        std::make_unique<comms::ContextRegistryService>(mgr_->registry()));
  }
  ~IrisGenerationApp() {
    mgr_->Stop();
    while (!mgr_thread_->joinable()) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(join_thread_sleep_msec_));
      logging::log()->debug(
          "IrisGenerationApp:Dtor: waiting for manager thread to become "
          "joinable...");
    }
    mgr_thread_->join();
    logging::log()->debug("IrisGenerationApp:Dtor: Joined manager thread");

    srv_->Shutdown();
  }

  void Run() {
    logging::log()->info("IrisGenerationApp:Run: Starting components");
    // start manager
    mgr_thread_ = std::make_unique<std::thread>(&IrisBuildManager::Run, mgr_);
    // start IRIS generation server
    srv_->Run();
  }

 private:
  const size_t join_thread_sleep_msec_ {100};
  const std::string addr_;
  std::unique_ptr<std::thread> mgr_thread_;
  std::shared_ptr<IrisBuildManager> mgr_;
  std::unique_ptr<comms::ServerWrapper> srv_;
};
