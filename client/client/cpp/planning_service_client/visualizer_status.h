/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file visualizer_status.h

#pragma once
#include <string>

namespace planning_service_client {

/**
 * @brief Represents the current status of a visualizer session.
 *
 * Unpacked from proto::GetVisualizerStatusResponse, containing both the
 * discrete status value and a human-readable details string.
 */
class VisualizerStatus {
 public:
  /** Discrete status values, mirroring proto::VisualizerStatus. */
  enum class Status {
    kUnspecified = 0,
    kIdle = 1,
    kActive = 2,
    kStarting = 3,
    kStopping = 4,
    kError = 5,
  };

  VisualizerStatus() = default;

  VisualizerStatus(Status status, std::string details)
      : status_(status), details_(std::move(details)) {}

  /** The discrete status of the visualizer. */
  Status status() const {
    return status_;
  }

  /** Human-readable details accompanying the status. */
  const std::string& details() const {
    return details_;
  }

 private:
  Status status_ {Status::kUnspecified};
  std::string details_;
};

}  // namespace planning_service_client
