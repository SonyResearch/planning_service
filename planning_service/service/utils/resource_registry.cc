/// @file resource_registry.cc

#include "resource_registry.h"

#include "planning_service/common/misc_utils.h"
#include "planning_service/service/utils/web/meshcat_ports.h"

namespace service {
namespace utils {

const fs::path ResourceRegistry::kDefaultLogPathRoot = "/logs";
const fs::path ResourceRegistry::kDefaultDataPathRoot = "/data";

ResourceRegistry::ResourceRegistry(const std::string& system_name,
                                   const ResourceOptions& options)
    : version_ {common::utils::GetTag()},
      system_name_ {system_name},
      options_ {options},
      data_path_root_ {fs::path(
          options_.data_path_root_override.value_or(kDefaultDataPathRoot))},
      data_path_ {data_path_root_ / system_name_},
      log_path_ {kDefaultLogPathRoot / system_name_},
      context_base_path_ {data_path_ / "contexts"},
      maybe_html_port_ {options.make_meshcat_ports_html_thread
                            ? std::make_optional(options.html_port)
                            : std::nullopt} {
  if (!fs::is_directory(context_base_path_)) {
    fs::create_directories(context_base_path_);
  }
  if (!options_.load_resources_on_init) {
    logging::log()->warn(
        "ResourceRegistry:Ctor: Skipping loading resources at "
        "initialization");
    return;
  }
  const auto available_contexts {
      LoadAvailableContexts(context_base_path_, options_)};
  std::map<uint64_t, std::future<void>> result_map {};
  std::map<uint64_t, std::string> error_map;
  for (const auto& context : available_contexts) {
    if (options_.load_in_parallel) {
      result_map.emplace(context.id->value,
                         std::async(std::launch::async, [this, &context] {
                           AddDraco(context.id->value, context);
                         }));
    } else {
      try {
        AddDraco(context.id->value, context);
      } catch (const std::exception& e) {
        error_map.emplace(context.id->value, e.what());
      }
    }
  }
  for (auto& [id, future] : result_map) {
    try {
      future.get();
    } catch (const std::exception& e) {
      error_map.emplace(id, e.what());
    }
  }
  if (!error_map.empty()) {
    std::string error_msg = "Failed to load some Draco instances:\n";
    for (const auto& [id, msg] : error_map) {
      error_msg += fmt::format("CID: {} ({})\n", id, msg);
    }
    logging::log()->error("ResourceRegistry:Ctor: {}", error_msg);
    if (options_.require_all_resources) {
      throw std::runtime_error(error_msg);
    }
  }
  if (options_.draco_is_builder) {
    int num_samples {0};
    for (const auto& [id, pdraco] : draco_map_) {
      logging::log()->info(
          "ID: {}\n{}", id,
          pdraco->mutable_artifact_builder().GetPlanningArtifactStatus(
              num_samples));
    }
  }
  const int num_successful {
      static_cast<int>(available_contexts.size() - error_map.size())};
  logging::log()->info(
      "ResourceRegistry:Ctor: Successfully loaded {}/{} Draco instance(s)",
      num_successful, available_contexts.size());
  // Make the meshcat port website
  LaunchOrResetResourcesHtmlThread();
}

void ResourceRegistry::AddDraco(const draco::PlanContextId& id,
                                const service::PlanContext& context) {
  auto hash {id.value};
  std::optional<draco::VisualizerMode> visualizer_mode;
  if (options_.make_draco_visualizers) {
    visualizer_mode = draco::VisualizerMode::kDraco;
  }
  const auto adapter {utils::MakeDracoAdapterFromContext(
      system_name_, context, options_.require_parameters, visualizer_mode)};
  auto planner {std::make_unique<draco::planner::DracoPlanner>(
      adapter, options_.draco_is_builder)};
  {
    std::scoped_lock<std::mutex> lock(draco_map_mutex_);
    const auto& computed_hash {planner->robot_constraints().constraints_hash()};
    if (computed_hash != hash) {
      if (!options_.update_resource_ids) {
        throw std::runtime_error(fmt::format(
            "Assigned hash [{}] != computed [{}]", hash, computed_hash));
      }
      logging::log()->warn(
          "ResourceRegistry:AddDraco: Overwriting assigned hash ({} -> {})",
          hash, computed_hash);
      std::filesystem::rename(
          context.context_dir,
          context.context_dir.parent_path() / std::to_string(computed_hash));
      hash = computed_hash;
    }
    if (options_.dry_run) {
      logging::log()->info(
          "ResourceRegistry:AddDraco: Dry run enabled, not storing Draco for "
          "context {}",
          hash);
      return;
    }
    draco_map_.emplace(hash, std::move(planner));
    active_contexts_map_.emplace(hash, context);
  }
}

void ResourceRegistry::RemoveDraco(const draco::PlanContextId& id) {
  const auto& hash {id.value};
  if (!draco_map_.count(hash)) {
    throw std::runtime_error(
        fmt::format("Hash requested for removal is not present! ({})", hash));
  }
  active_contexts_map_.erase(hash);
  draco_map_.erase(hash);
}

std::expected<draco::PlanContextId, ServiceError>
ResourceRegistry::RegisterPlanContext(const std::string& system,
                                      PlanContext context) {
  logging::log()->info("Received registration request for {}!", system);
  const auto data_path {data_path_root_ / system};
  const auto context_base_path {data_path / "contexts"};
  if (!context.id.has_value()) {
    // 1. Compute unique ID
    const auto urdf_dir {data_path / "urdf"};
    const auto context_id_opt {utils::RegisterPlanContext(context, urdf_dir)};
    if (!context_id_opt) {
      return std::unexpected(ServiceError(ServiceErrorCode::CONTEXT_INVALID,
                                          "Failed to compute context ID!"));
    }
    const auto context_id {*context_id_opt};
    context.id = context_id;
    context.urdf_dir = urdf_dir;
    // 3. Save context
    if (!utils::SaveContext(context, context_base_path)) {
      return std::unexpected(ServiceError(ServiceErrorCode::CONTEXT_INVALID,
                                          "Failed to save context to disk!"));
    }
  } else {
    if (!utils::LoadContext(context, context_base_path)) {
      return std::unexpected(ServiceError(ServiceErrorCode::CONTEXT_INVALID,
                                          "Failed to load context from disk!"));
    }
    logging::log()->info(
        "RR:RegisterPlanContext: Loaded existing context from: {}",
        context.context_dir);
  }
  const auto context_id {*context.id};
  // Only enable the Draco if it is not registered and part of the active system
  if (system != system_name_) {
    logging::log()->warn(
        "ResourceRegistry:RegisterPlanContext: Saved context {} is not "
        "part of the active system {}. Will not be available for planning.",
        context_id, system_name_);
    return context_id;
  }
  if (!HasDraco(context_id)) {
    try {
      // 5. make draco
      AddDraco(context_id, context);
    } catch (const std::exception& e) {
      return std::unexpected(ServiceError(
          ServiceErrorCode::CONTEXT_INVALID,
          fmt::format("Failed to create Draco instance due to exception: {}",
                      e.what())));
    }
  }
  return context_id;
}

std::expected<draco::PlanContextId, ServiceError>
ResourceRegistry::RemovePlanContext(const std::string& system,
                                    const draco::PlanContextId& context_id,
                                    bool erase) {
  if (system != system_name_) {
    const auto err_msg {
        fmt::format("Context {} is not part of the active system {}. Cannot "
                    "remove context!",
                    context_id, system_name_)};
    logging::log()->error("ResourceRegistry:RemovePlanContext: {}", err_msg);
    return std::unexpected(
        ServiceError(ServiceErrorCode::CONTEXT_INVALID, err_msg));
  }
  if (!HasDraco(context_id)) {
    const auto err_msg {fmt::format(
        "Context {} not found in active system {}. Cannot remove context!",
        context_id, system_name_)};
    logging::log()->error("ResourceRegistry:RemovePlanContext: {}", err_msg);
    return std::unexpected(
        ServiceError(ServiceErrorCode::CONTEXT_NOT_FOUND, err_msg));
  }
  if (erase) {
    const auto context_path {context_base_path_
                             / std::to_string(context_id.value)};
    if (!fs::is_directory(context_path)) {
      const auto err_msg {fmt::format(
          "Context path {} not found on disk. Cannot erase context!",
          context_path)};
      logging::log()->error("ResourceRegistry:RemovePlanContext: {}", err_msg);
      return std::unexpected(
          ServiceError(ServiceErrorCode::CONTEXT_NOT_FOUND, err_msg));
    }
    fs::remove_all(context_path);
    logging::log()->info(
        "ResourceRegistry:RemovePlanContext: Erased context {} from disk at "
        "{}",
        context_id, context_path);
  }
  RemoveDraco(context_id);
  active_contexts_map_.erase(context_id.value);
  return context_id;
}

std::expected<void, std::string> ResourceRegistry::MigratePlanningArtifacts(
    const draco::PlanContextId& from_context_id,
    const draco::PlanContextId& to_context_id, const int num_samples,
    const bool repair_artifacts) {
  // This function assumes that the resource registry is a builder
  if (!options_.draco_is_builder) {
    return std::unexpected(
        "This ResourceRegistry was not initialized as a builder. Cannot "
        "migrate planning artifacts!");
  }
  const std::string from_context_id_str {std::to_string(from_context_id.value)};
  const std::string to_context_id_str {std::to_string(to_context_id.value)};
  const auto from_context_path {context_base_path() / from_context_id_str};
  const auto to_context_path {context_base_path() / to_context_id_str};
  // check if from context exists
  if (!fs::is_directory(from_context_path)) {
    return std::unexpected(fmt::format(
        "Directory for source {} does not exist!", from_context_id));
  }
  if (!HasDraco(from_context_id)) {
    return std::unexpected(
        fmt::format("No Draco instance found for source {}!", from_context_id));
  }
  if (!fs::is_directory(to_context_path)) {
    return std::unexpected(
        fmt::format("Directory for target {} does not exist!", to_context_id));
  }
  // if from context is not the same as to context, copy the planning artifacts
  if (from_context_id != to_context_id) {
    // copy the iris regions file
    if (!fs::is_regular_file(from_context_path / IRIS_REGIONS_FILE)) {
      logging::log()->warn(
          "ResourceRegistry:MigratePlanningArtifacts: Source context {} does "
          "not have planning artifacts to migrate.",
          from_context_id);
    } else {
      fs::copy(from_context_path / IRIS_REGIONS_FILE,
               to_context_path / IRIS_REGIONS_FILE,
               fs::copy_options::overwrite_existing);
      logging::log()->info(
          "ResourceRegistry:MigratePlanningArtifacts: Copied planning "
          "artifacts from context {} to context {}",
          from_context_id, to_context_id);
    }
  }
  // Check that draco instance exists for to_context_id, if so, migrate the
  // planning artifacts
  // remove Draco instance and then load it again
  PlanContext to_context {to_context_id.value};
  if (HasDraco(to_context_id)) {
    RemoveDraco(to_context_id);
  }
  utils::LoadContext(to_context, context_base_path());
  AddDraco(to_context_id, to_context);
  // remove roadmap files after loading them into the new context, and to avoid
  // any saving issues if files still exist
  fs::remove(to_context_path / THUNDER_ROADMAP_FILE);
  logging::log()->info(
      "ResourceRegistry:MigratePlanningArtifacts: Migrating roadmap from "
      "context {} to context {}",
      from_context_id, to_context_id);
  GetMutableDraco(to_context_id)
      ->mutable_artifact_builder()
      .LoadValidateAndSaveFiles(from_context_path / THUNDER_ROADMAP_FILE,
                                to_context_path / THUNDER_ROADMAP_FILE,
                                repair_artifacts);  // only repair if required
  logging::log()->info(
      "ResourceRegistry:MigratePlanningArtifacts: Migrating regions from "
      "context {} to context {}. Repairing regions is set to {} with {} "
      "samples",
      from_context_id, to_context_id, repair_artifacts, num_samples);
  if (repair_artifacts) {
    fs::remove(to_context_path / IRIS_REGIONS_FILE);
    GetMutableDraco(to_context_id)
        ->mutable_artifact_builder()
        .RepairRegions({}, num_samples);
  }
  RemoveDraco(to_context_id);
  utils::LoadContext(to_context, context_base_path());
  AddDraco(to_context_id, to_context);
  // Reset the HTML thread to update the ports
  LaunchOrResetResourcesHtmlThread();
  return {};
}

fs::path ResourceRegistry::NewProblemDirectory(
    const draco::PlanContextId& context_id, const std::string& plan_id) const {
  const auto path {
      log_path_ / "problems"
      / common::utils::datetime_str(common::utils::DATE_FMT)
      / std::to_string(context_id.value)
      / fmt::format("{}-{}", plan_id.empty() ? "unidentified" : plan_id,
                    common::utils::datetime_str(common::utils::DATETIME_FMT))};
  if (!fs::is_directory(path)) {
    fs::create_directories(path);
  }
  return path;
}

void ResourceRegistry::LaunchOrResetResourcesHtmlThread() {
  if (!maybe_html_port_.has_value()) {
    logging::log()->info(
        "ResourceRegistry:LaunchOrResetResourcesHtmlThread: No HTML port "
        "specified. Skipping HTML thread launch.");
    return;
  }
  logging::log()->info(
      "ResourceRegistry:LaunchOrResetResourcesHtmlThread: Launching HTML "
      "thread on port {}",
      maybe_html_port_.value());
  // Reset the html thread
  html_stop_requested_ = true;
  html_cv_.notify_all();
  // Restart the HTML thread to update the ports
  if (html_thread_.joinable()) {
    html_thread_.join();
  }
  html_stop_requested_ = false;
  html_thread_ =
      std::thread(&web::MakeMeshcatPortsHtml, std::ref(draco_map_),
                  std::ref(html_stop_requested_), std::ref(html_mutex_),
                  std::ref(html_cv_), maybe_html_port_.value());
}

}  // namespace utils
}  // namespace service
