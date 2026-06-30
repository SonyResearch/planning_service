#include "iris_manager.h"

#include <magic_enum/magic_enum.hpp>
namespace service {
namespace iris {

using draco::PlanContextId;
using draco::PlanningProblemDef;

const std::expected<bool, ServiceError> IrisBuildManager::StartJobImpl(
    const IrisBuildRequestAdapter& req) {
  logging::log()->info("IBM:StartJobImpl: Starting job {}", req.id);
  if (*req.context.id == 0) {
    logging::log()->error("IBM:StartJobImpl: Invalid context provided!");
    return std::unexpected(ServiceError(ServiceErrorCode::CONTEXT_INVALID,
                                        "Context did not contain a valid ID"));
  }
  // 6. Assign and start the job
  job_runner_->InsertNewJob(req.id, std::bind(&IrisBuildManager::BuildRegions,
                                              this, *req.context.id, req));
  return true;
}

const IrisBuildResult IrisBuildManager::BuildRegionsFromSystemConfs(
    const PlanContextId& context_id,
    const std::vector<system_conf_t>& sysconf_vec) {
  logging::log()->info(
      "IBM:BuildRegionsFromSystemConfs: Starting job for context {}",
      context_id);
  // convert system configurations to Eigen::VectorXd
  std::vector<Eigen::VectorXd> conf_vec;
  for (const auto& sysconf : sysconf_vec) {
    conf_vec.push_back(registry_->GetMutableDraco(context_id)
                           ->robot_model()
                           .ToGeneralizedPosition(sysconf));
  }
  registry_->GetMutableDraco(context_id)
      ->mutable_artifact_builder()
      .BuildRegionsFromSystemConfs(conf_vec);
  return IrisBuildResult(context_id, IrisBuildJobType::IRIS_FROM_CONFIGS);
}

const IrisBuildResult IrisBuildManager::BuildRegionsFromEdges(
    const PlanContextId& context_id,
    const std::vector<system_conf_edge_t>& sysconf_edge_vec) {
  logging::log()->info("IBM:BuildRegionsFromEdges: Starting job for context {}",
                       context_id);
  // convert system configuration edges to Eigen::VectorXd
  std::vector<motion::iris::conf_edge_t> conf_edge_vec;
  for (const auto& [u, v] : sysconf_edge_vec) {
    const auto& q_u {registry_->GetDraco(context_id)
                         ->robot_model()
                         .ToGeneralizedPosition(u)};
    const auto& q_v {registry_->GetDraco(context_id)
                         ->robot_model()
                         .ToGeneralizedPosition(v)};
    conf_edge_vec.emplace_back(q_u, q_v);
  }
  registry_->GetMutableDraco(context_id)
      ->mutable_artifact_builder()
      .BuildRegionsFromEdges(conf_edge_vec);
  return IrisBuildResult(context_id, IrisBuildJobType::IRIS_FROM_EDGES);
}

const IrisBuildResult IrisBuildManager::BuildRegionsFromRoadmap(
    const PlanContextId& context_id, const int num_problems) {
  logging::log()->info(
      "IBM:BuildRegionsFromRoadmap: Starting job for context {}", context_id);
  // Build regions from the planning problems
  registry_->GetMutableDraco(context_id)
      ->mutable_artifact_builder()
      .BuildRegionsFromSavedProblemsPath(num_problems);
  // Build regions from the roadmap
  registry_->GetMutableDraco(context_id)
      ->mutable_artifact_builder()
      .BuildRegionsFromRoadmap();
  return IrisBuildResult(context_id, IrisBuildJobType::IRIS_FROM_ROADMAP);
}

void IrisBuildManager::GenerateSamplesAndSolveProblems(
    const PlanContextId& context_id, const int num_desired_samples) {
  logging::log()->info(
      "IBM:GenerateSamplesAndSolveProblems: Starting job for context {}",
      context_id);
  const auto& draco {registry_->GetDraco(context_id)};
  drake::RandomGenerator generator {650};
  const auto samples {draco->robot_constraints().GenerateSamples(
      &generator, num_desired_samples)};
  // number of returned samples can be less than number desired
  const auto num_samples {
      std::min(num_desired_samples, static_cast<int>(samples.size()))};
  int counter = 0;
  auto t_now {hr_clock::now()};
  const uint16_t logging_interval_ms {5000};
  const int num_problems {num_samples * (num_samples - 1) / 2};
  logging::log()->info(
      "IBM:GenerateSamplesAndSolveProblems: Attempting to solve {} problems...",
      num_problems);
  for (int i = 0; i < num_samples; ++i) {
    const auto& q_start {samples.at(i)};
    for (int j = i + 1; j < num_samples; ++j) {
      const auto& q_goal {samples.at(j)};
      const bool try_recall {true};
      const bool save_solution {true};
      draco->mutable_artifact_builder().GetSampleBasedSolution(
          q_start, q_goal, try_recall, save_solution);
      ++counter;
      if (const auto t_new_now {hr_clock::now()};
          std::chrono::duration_cast<chrono_ms>(t_new_now - t_now).count()
          >= logging_interval_ms) {
        t_now = t_new_now;
        logging::log()->info(
            "IBM:GenerateSamplesAndSolveProblems: Solved {}/{} problems...",
            counter, num_problems);
      }
    }
  }
  logging::log()->info(
      "IBM:GenerateSamplesAndSolveProblems: Solved all {} problems.",
      num_problems);
}

const std::optional<UpdateRoadmapResult>
IrisBuildManager::UpdateRoadmapFromProblems(
    const PlanContextId& context_id, const UpdateRoadmapRequestAdapter& req) {
  logging::log()->info(
      "IBM:UpdateRoadmap: Starting job for context {} from problems",
      context_id);
  std::vector<PlanningProblemDef> problems;
  const auto& pdraco {registry_->GetMutableDraco(context_id)};
  switch (req.job_type) {
    case UpdateRoadmapJobType::UPDATE_FROM_SAVED_PROBLEMS: {
      // ToDo(@Sadra): This is deprecated. We should refactor this to use the
      // new API.
      pdraco->mutable_artifact_builder().UpdateRoadmapFromSavedProblems(
          req.num_problems);
      break;
    }
    case UpdateRoadmapJobType::UPDATE_FROM_PASSED_PROBLEMS: {
      problems = req.planning_problem_vec;
      break;
    }
    default:
      break;  // no problems to solve
  }

  if (problems.empty()) {
    logging::log()->error(
        "IBM:UpdateRoadmapFromProblems: No passed problems to solve!");
    return std::nullopt;
  }
  for (const auto& problem : problems) {
    logging::log()->warn(
        "IBM:UpdateRoadmapFromProblems: The usage of old problems method is "
        "deprecated. "
        "Please refactor to use the new API to solve problems.");
    try {
      const bool save_solution {true};
      const bool try_recall_planner {false};
      pdraco->mutable_artifact_builder().GetSampleBasedSolution(
          problem.start_conf, problem.goal_conf, try_recall_planner,
          save_solution);
    } catch (const std::exception& e) {
      logging::log()->error("URM:StartJobImpl: Job failed due to exception: {}",
                            e.what());
      continue;
    }
  }
  logging::log()->info(
      "IBM:UpdateRoadmap: Finished updating roadmap from specified planning "
      "problems for context {}",
      context_id);
  return UpdateRoadmapResult(context_id);
}

const std::optional<UpdateRoadmapResult>
IrisBuildManager::UpdateRoadmapFromSamples(
    const PlanContextId& context_id, const UpdateRoadmapRequestAdapter& req) {
  GenerateSamplesAndSolveProblems(context_id, req.num_samples);
  logging::log()->info(
      "IBM:UpdateRoadmap: Finished updating roadmap from generated samples for "
      "context {}",
      context_id);
  return UpdateRoadmapResult(context_id);
}

const std::optional<UpdateRoadmapResult> IrisBuildManager::UpdateRoadmap(
    const PlanContextId& context_id, const UpdateRoadmapRequestAdapter& req) {
  const auto& job_type {req.job_type};
  logging::log()->info(
      "IBM:UpdateRoadmap: Starting job for context {} with job type {}",
      context_id, magic_enum::enum_name(job_type));
  UpdateRoadmapFromSamples(context_id, req);
  UpdateRoadmapFromProblems(context_id, req);
  return UpdateRoadmapResult(context_id);
}
}  // namespace iris
}  // namespace service
