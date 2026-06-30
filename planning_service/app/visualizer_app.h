/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file iris_builder_app.h

#pragma once
#include "planning_service/comms/server_wrapper.h"
#include "planning_service/comms/viz_service.h"

class VisualizerApp {
 public:
  /**
   * @brief Construct a new IRIS Generation App
   *
   * @param addr Address at which the IRIS builder will listen for new
   * requests
   * @param system_name Enabled system name
   */
  VisualizerApp(const std::string& addr, const std::string& system_name,
                const std::optional<service::visualization::VisualizerOptions>&
                    options = std::nullopt)
      : addr_ {addr} {
    hub_ = std::make_shared<service::visualization::VisualizerHub>(system_name,
                                                                   options);
    srv_ = std::make_shared<comms::ServerWrapper>(addr_);
    srv_->AddService(std::make_unique<comms::VisualizerService>(hub_));
  }

  ~VisualizerApp() {
    srv_->Shutdown();
    while (!srv_thread_->joinable()) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(join_thread_sleep_msec));
      logging::log()->debug(
          "VisualizerApp:Dtor: waiting for server thread to become "
          "joinable...");
    }
    srv_thread_->join();
    logging::log()->debug("VisualizerApp:Dtor: Joined server thread");
  }

  void Run() {
    logging::log()->info("VisualizerApp:Run: Starting server");
    // start visualizer server
    srv_thread_ =
        std::make_unique<std::thread>(&comms::ServerWrapper::Run, srv_);
    hub_->Run();
  }

 private:
  const std::string addr_;
  const uint8_t join_thread_sleep_msec {100};
  std::shared_ptr<service::visualization::VisualizerHub> hub_;
  std::shared_ptr<comms::ServerWrapper> srv_;
  std::unique_ptr<std::thread> srv_thread_;
};
