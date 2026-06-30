/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file types.h

#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <drake/common/polynomial.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

#include <future>
#include <variant>

#include <nlohmann/json.hpp>

#include "planning_service/draco/draco.h"

namespace fs = std::filesystem;
using chrono_ms = std::chrono::milliseconds;
using hr_clock = std::chrono::high_resolution_clock;
using json = nlohmann::json;

namespace draco {
// @warning: This struct is deprecated.
struct PlanningProblemDef {
  Eigen::VectorXd start_conf, goal_conf;
};

/**
 * @brief Struct containing unique ID for a given plan context (geometry +
 * constraints). This is also intended to capture other relevant data as deemed
 * necessary (f.e., the name of the given system).
 *
 */
struct PlanContextId {
  PlanContextId() = default;

  PlanContextId(uint64_t value) : value {value} {}

  // Operator overload for ==
  bool operator==(const PlanContextId& other) const {
    return value == other.value;
  }

  // Operator overload for ==
  bool operator==(uint64_t other) const {
    return value == other;
  }

  uint64_t value;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }
};

}  // namespace draco

namespace service {

using system_conf_t = std::map<std::string, Eigen::VectorXd>;
using system_conf_edge_t = std::pair<system_conf_t, system_conf_t>;

using request_id_t = std::string;  // gRPC request ID

/** A file containing a mesh. */
struct MeshFile {
  std::string name;
  std::string extension;  // e.g. ".obj", ".dae", ".stl"
  std::string package_name;
  fs::path parent_path;
  bool collision;  // visual if false
  std::string content;
};
enum ModelFormat { URDF };
/** A file containing the geometry of a single model instance. */
struct ModelFile {
  std::string name;
  ModelFormat format;
  std::string package_name;
  fs::path parent_path;
  std::string content;
};

/**
 * @brief Represents a single model instance referenced in the complete scene.
 * Contains a name and some representation of the geometry, plus some additional
 * configuration data.
 *
 */
struct Model {
  std::string name;
  ModelFile file;
  std::vector<MeshFile> meshes;
  std::map<std::string, Eigen::VectorXd> default_joint_positions;
  std::map<std::string, drake::math::RigidTransformd> default_free_body_pose;
};

/**
 * @brief All information required to uniquely define a the set of geometry and
 * constraints which are used to construct a single IRIS builder or planner
 * instance.
 *
 */
struct PlanContext {
  std::string name;
  std::string system;
  drake::multibody::parsing::ModelDirectives model_directive;
  std::vector<Model> models;
  motion::ConstraintsAdapter constraints_adapter;
  std::optional<draco::PlanContextId> id {std::nullopt};
  fs::path context_dir {""};  // dir where context YAMLs are saved
  fs::path urdf_dir {""};     // dir where URDFs are saved (may be different)
  json metadata;              // additional context data
  PlanContext() = delete;
  PlanContext(const std::string& name,
              const drake::multibody::parsing::ModelDirectives& model_directive,
              const std::vector<Model>& models,
              const motion::ConstraintsAdapter& constraints_adapter,
              const std::optional<json>& metadata = std::nullopt)
      : name {name},
        model_directive {model_directive},
        models {models},
        constraints_adapter {constraints_adapter} {
    if (metadata) {
      this->metadata = *metadata;
    }
  }
  PlanContext(const uint64_t id) : id {draco::PlanContextId(id)} {}
  PlanContext(const fs::path& context_dir) : context_dir {context_dir} {}
};

/** Abstract request class, the input of a job to be assigned to JobRunner. */
struct RequestAdapter {
  request_id_t id;
  explicit RequestAdapter(const request_id_t& id) : id {id} {}
  virtual ~RequestAdapter() = 0;
};
inline RequestAdapter::~RequestAdapter() {}

namespace iris {
enum class IrisBuildJobType {
  NONE = 0,
  IRIS_FROM_CONFIGS = 1,
  IRIS_FROM_EDGES = 2,
  IRIS_FROM_ROADMAP = 3,
  IRIS_FROM_PROBLEMS = 4
};

// enum class to specify the type of job to be executed to update the roadmap
enum class UpdateRoadmapJobType {
  NONE = 0,
  UPDATE_FROM_SAVED_PROBLEMS = 1,
  UPDATE_FROM_PASSED_PROBLEMS = 2,
  UPDATE_FROM_SAMPLES = 3
};

/**
 * @brief Container for ongoing IRIS thread and statistics
 *
 * TODO(@davebambrick): add stats of ongoing job (% coverage, etc.)
 */
struct IrisBuildResult {
  const draco::PlanContextId context_id;
  const IrisBuildJobType type {IrisBuildJobType::NONE};
  explicit IrisBuildResult(const draco::PlanContextId& context_id,
                           const IrisBuildJobType& type)
      : context_id {context_id}, type {type} {}
};

/**
 * @brief Struct to be populated from gRPC IRIS build requests processed from
 * a corresponding server instance. Contains all necessary information to
 * define a unique IRIS region and data with which to populate it.
 *
 */
struct IrisBuildRequestAdapter final : public RequestAdapter {
  const PlanContext context;
  int num_problems {0};
  const IrisBuildJobType job_type {IrisBuildJobType::NONE};
  const std::vector<system_conf_t> sysconf_vec;
  const std::vector<system_conf_edge_t> sysconf_edge_vec;
  const std::vector<draco::PlanningProblemDef> planning_problem_vec;
  // Sysconf constructor
  explicit IrisBuildRequestAdapter(
      const request_id_t& id, const PlanContext& context,
      const int num_problems, const IrisBuildJobType job_type,
      const std::vector<system_conf_t>& sysconf_vec,
      const std::vector<system_conf_edge_t>& sysconf_edge_vec,
      const std::vector<draco::PlanningProblemDef>& planning_problem_vec)
      : RequestAdapter(id),
        context {context},
        num_problems {num_problems},
        job_type {job_type},
        sysconf_vec {sysconf_vec},
        sysconf_edge_vec {sysconf_edge_vec},
        planning_problem_vec {planning_problem_vec} {}
};

struct UpdateRoadmapResult {
  const draco::PlanContextId context_id;
  explicit UpdateRoadmapResult(const draco::PlanContextId& context_id)
      : context_id {context_id} {}
};

struct UpdateRoadmapRequestAdapter final : public RequestAdapter {
  const PlanContext context;
  const UpdateRoadmapJobType job_type {UpdateRoadmapJobType::NONE};
  const int num_samples;
  const int num_problems;
  const int num_fpp_problems;
  const int num_random_ik_seed_samples;
  const bool insert_solution_in_roadmap;
  const std::vector<draco::PlanningProblemDef> planning_problem_vec;

  explicit UpdateRoadmapRequestAdapter(
      const request_id_t& id, const PlanContext& context,
      const UpdateRoadmapJobType job_type, const int num_samples,
      const int num_problems, const int num_fpp_problems,
      const int num_random_ik_seed_samples,
      const bool insert_solution_in_roadmap,
      const std::vector<draco::PlanningProblemDef>& planning_problem_vec)
      : RequestAdapter(id),
        context {context},
        job_type {job_type},
        num_samples {num_samples},
        num_problems {num_problems},
        num_fpp_problems {num_fpp_problems},
        num_random_ik_seed_samples {num_random_ik_seed_samples},
        insert_solution_in_roadmap {insert_solution_in_roadmap},
        planning_problem_vec {planning_problem_vec} {}
};

struct MigratePlanningArtifactsRequestAdapter final : public RequestAdapter {
  const draco::PlanContextId from_context_id;
  const draco::PlanContextId to_context_id;
  const int num_samples;
  const bool repair_artifacts;

  explicit MigratePlanningArtifactsRequestAdapter(
      const request_id_t& id, const draco::PlanContextId& from_context_id,
      const draco::PlanContextId& to_context_id, const int num_samples,
      const bool repair_artifacts = true)
      : RequestAdapter(id),
        from_context_id {from_context_id},
        to_context_id {to_context_id},
        num_samples {num_samples},
        repair_artifacts {repair_artifacts} {}
};

}  // namespace iris

}  // namespace service

/// \cond DO_NOT_DOCUMENT
template <>
struct fmt::formatter<draco::PlanContextId> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(draco::PlanContextId const& id, FormatContext& ctx) const {
    return fmt::format_to(ctx.out(), "{}", id.value);
  }
};
/// \endcond
