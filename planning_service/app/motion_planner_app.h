/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file app.h

#pragma once

#include "planning_service/comms/plan_service.h"
#include "planning_service/comms/registry_service.h"
#include "planning_service/comms/server_wrapper.h"

using service::planning::MotionPlanManager;
class MotionPlannerApp {
 public:
  /**
   * @brief Construct a new Motion Planner App object
   *
   * @param addr Address at which the motion planner will listen for
   * new plan requests
   * @param system_name Enabled system name
   */
  MotionPlannerApp(const std::string& addr, const std::string& system_name,
                   const service::utils::ResourceOptions& options)
      : addr_ {addr} {
    // start clock
    const auto start_time {std::chrono::high_resolution_clock::now()};
    mgr_ = std::make_shared<MotionPlanManager>(system_name, options);
    // Create server and add services
    srv_ = std::make_unique<comms::ServerWrapper>(addr_);
    srv_->AddService(std::make_unique<comms::MotionPlannerService>(mgr_));
    srv_->AddService(
        std::make_unique<comms::ContextRegistryService>(mgr_->registry()));
    logging::log()->info(
        "MotionPlannerApp:Ctor: Finished initializing services in {:.3f}s",
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start_time)
                .count()
            * 1e-3);
  }
  ~MotionPlannerApp() {
    srv_->Shutdown();
  }
  /** Start all modules. */
  void Run() {
    logging::log()->info("MotionPlannerApp:Run: Starting components");
    // start planning server
    srv_->Run();
  }

 private:
  const size_t join_thread_sleep_msec_ {100};
  const std::string addr_;
  std::shared_ptr<MotionPlanManager> mgr_;
  std::unique_ptr<comms::ServerWrapper> srv_;
};
