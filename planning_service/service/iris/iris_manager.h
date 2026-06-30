/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#pragma once

#include "planning_service/service/types/types.h"
#include "planning_service/service/utils/resource_manager.h"
#include "planning_service/service/utils/utils.h"

namespace fs = std::filesystem;

namespace service {
namespace iris {

/**
 * @brief Class which manages threads of computation for generating IRIS
 * regions.
 */
class IrisBuildManager
    : public utils::ResourceManager<IrisBuildRequestAdapter, IrisBuildResult> {
 public:
  utils::ResourceOptions MakeBuilderOptions(
      const utils::ResourceOptions& options) {
    utils::ResourceOptions opts {options};
    opts.draco_is_builder = true;  // IRIS builder is a Draco builder
    return opts;
  }

  /**
   * @brief Constructor
   * @param system_name Enabled system name
   * @param options Options handling the loading of resources from disk
   * @param max_queue_size maximum number of requests which may wait in the
   * queue at a given instant
   * @param max_active_jobs maximum number of IRIS jobs which may be running at
   * a given instant
   */
  IrisBuildManager(const std::string& system_name,
                   const utils::ResourceOptions& options,
                   const size_t max_queue_size = 1,
                   const size_t max_active_jobs = 1)
      : utils::ResourceManager<IrisBuildRequestAdapter, IrisBuildResult>(
            system_name, MakeBuilderOptions(options), max_queue_size,
            max_active_jobs) {}

  /**
   * @brief Update the sample-based roadmap given the input request.
   *
   * @param context_id
   * @param req
   * @return const std::optional<UpdateRoadmapResult>
   *
   * TODO(@davebambrick, @yammineramy): Refactor IRIS manager to treat roadmap
   * and IRIS jobs the same.c
   */
  const std::optional<UpdateRoadmapResult> UpdateRoadmap(
      const draco::PlanContextId& context_id,
      const UpdateRoadmapRequestAdapter& req);

 protected:
  void Cleanup() {
    for (const auto& id : job_runner_->GetCompletedJobIds()) {
      const auto result {job_runner_->RetrieveResult(id)};
      const auto msg {fmt::format("IrisBuildManager:Cleanup: Job {} {}!", id,
                                  result.has_value() ? "succeeded" : "failed")};
      if (result.has_value()) {
        logging::log()->info(msg);
      } else {
        logging::log()->error(msg);
      }
    }
  }
  /**
   * @brief Start a thread to generate IRIS regions for the given model and
   * constraints using the seed configurations provided.
   *
   * @param req Request to start a new build job
   *
   * @return true on success, one of ServiceError on failure
   */
  const std::expected<bool, ServiceError> StartJobImpl(
      const IrisBuildRequestAdapter& req);

  /**
   * @brief Build IRIS regions from a set of system configs for the targeted
   * planning context.
   *
   * A thin wrapper around `IrisBuilder::BuildFromConfigs` which is intended to
   * be assigned to an async thread.
   *
   * @param context_id unique identifier which corresponds to a given planning
   * context
   * @param sysconf_vec A set of configurations from which the polytopes
   * will be generated
   *
   * @return true on success, one of ServiceError on failure
   */
  const IrisBuildResult BuildRegionsFromSystemConfs(
      const draco::PlanContextId& context_id,
      const std::vector<system_conf_t>& sysconf_vec);

  /**
   * @brief Build IRIS regions from a set of system config edges for the
   * targeted planning context.
   *
   * A thin wrapper around `IrisBuilder::BuildFromEdges` which is intended to
   * be assigned to an async thread.
   *
   * @param context_id unique identifier which corresponds to a given planning
   * context
   * @param sysconf_edge_vec A set of edges from which the polytopes
   * will be generated
   *
   * @return true on success, one of ServiceError on failure
   */
  const IrisBuildResult BuildRegionsFromEdges(
      const draco::PlanContextId& context_id,
      const std::vector<system_conf_edge_t>& sysconf_edge_vec);

  /**
   * @brief Build IRIS regions from the local RRT-based probabilistic roadmap,
   * by breaking it down into edges. The built regions will cover all of the
   * roadmap's edges.
   */
  const IrisBuildResult BuildRegionsFromRoadmap(
      const draco::PlanContextId& context_id, const int num_problems = 0);

  /** Method passed to InsertNewJob. */
  const std::expected<IrisBuildResult, ServiceError> BuildRegions(
      const draco::PlanContextId& context_id,
      const IrisBuildRequestAdapter& req) {
    try {
      switch (req.job_type) {
        case IrisBuildJobType::IRIS_FROM_EDGES: {
          return BuildRegionsFromEdges(context_id, req.sysconf_edge_vec);
        }
        case IrisBuildJobType::IRIS_FROM_CONFIGS: {
          return BuildRegionsFromSystemConfs(context_id, req.sysconf_vec);
          break;
        }
        case IrisBuildJobType::IRIS_FROM_ROADMAP: {
          return BuildRegionsFromRoadmap(context_id, req.num_problems);
        }
        default:
          return std::unexpected(ServiceError(
              ServiceErrorCode::JOB_TYPE_UNSUPPORTED,
              fmt::format("Job type {} is not currently supported!",
                          magic_enum::enum_name(req.job_type))));
      }
    } catch (const std::exception& e) {
      return std::unexpected(ServiceError(
          ServiceErrorCode::JOB_THREAD_EXCEPTION,
          fmt::format("Job failed due to exception: {}", e.what())));
    }
  }

  const std::optional<UpdateRoadmapResult> UpdateRoadmapFromProblems(
      const draco::PlanContextId& context_id,
      const UpdateRoadmapRequestAdapter& req);

  const std::optional<UpdateRoadmapResult> UpdateRoadmapFromSamples(
      const draco::PlanContextId& context_id,
      const UpdateRoadmapRequestAdapter& req);

  /**
   * @brief Generate samples, create planning problems, solve them, and add
   * solutions to database if not recalled.
   * @param context_id unique identifier which corresponds to a given planning
   * context
   * @param num_desired_samples number of samples to generate
   *
   */
  void GenerateSamplesAndSolveProblems(const draco::PlanContextId& context_id,
                                       const int num_desired_samples);
};

}  // namespace iris
}  // namespace service
