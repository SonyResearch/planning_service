/// @file utils.cc

#include "utils.h"

#include <fstream>

#include <magic_enum/magic_enum.hpp>

#include "planning_service/common/file_utils.h"
#include "planning_service/common/string_utils.h"

namespace service {
namespace utils {

using motion::RobotConstraints;
using IrisRegionsAdapter = motion::iris::IrisRegionsAdapter;
using draco::PlanContextId;
using motion::splining::cartesian_dynamic_limits_map_t;
using motion::splining::joint_dynamic_limits_map_t;
using motion::splining::TimeOptimalSplineParams;

bool LoadContext(PlanContext& context, const fs::path& context_base_dir,
                 const std::optional<fs::path>& urdf_base_dir) {
  if (!context.id.has_value() && context.context_dir.empty()) {
    logging::log()->error(
        "LoadContext: Cannot load context without knowing its hash or "
        "location!");
    return false;
  }
  const auto context_dir {context.id.has_value()
                              ? context_base_dir
                                    / std::to_string(context.id->value)
                              : context.context_dir};
  const auto context_id {
      context.id.has_value()
          ? context.id.value()
          : PlanContextId(std::stoull(context.context_dir.stem().string()))};
  logging::log()->info("LoadContext: Loading context ID from path {}",
                       context_dir);
  // Load the model information
  drake::multibody::parsing::ModelDirectives model_directive;
  motion::ConstraintsAdapter constraints_adapter;
  try {
    model_directive =
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            context_dir / CONTEXT_DMD_FILE);
    constraints_adapter = drake::yaml::LoadYamlFile<motion::ConstraintsAdapter>(
        context_dir / CONTEXT_CONSTRAINTS_FILE);
  } catch (const std::exception& e) {
    logging::log()->info(
        "LoadContext: Attempted to load context from disk at path "
        "{}, but failed due to exception: {}",
        context_dir, e.what());
    return false;
  }
  // Load metadata
  const auto metadata_file {context_dir / CONTEXT_METADATA_FILE};
  json metadata;
  if (fs::is_regular_file(metadata_file)) {
    logging::log()->debug("LoadContext: Loading metadata from disk at {}",
                          metadata_file);
    std::ifstream f {metadata_file, std::fstream::binary | std::fstream::in};
    if (f.is_open()) {
      try {
        metadata = json::parse(f);
        logging::log()->debug("LoadContext: Loaded metadata from disk: {}",
                              metadata.dump());
      } catch (const std::exception& e) {
        logging::log()->error("LoadContext: Failed to load metadata from disk");
      }
    }
  }
  if (metadata.empty()) {
    // default name
    auto name {
        fmt::format("unnamed_context-{}", std::to_string(context_id.value))};
    metadata["scene_name"] = name;
    // save metadata
    common::utils::SaveJsonToFile(metadata_file, metadata);
  }
  const auto base_dir {context_dir.parent_path().parent_path()};
  context.urdf_dir = urdf_base_dir.value_or(base_dir / "urdf");
  context.context_dir = context_dir;
  context.id = PlanContextId {context_id};
  context.model_directive = model_directive;
  context.constraints_adapter = constraints_adapter;
  context.metadata = metadata;
  context.name = context.metadata["scene_name"];
  return true;
}

bool SaveContext(PlanContext& context, const fs::path& context_base_dir) {
  if (!context.id.has_value()) {
    logging::log()->error(
        "SaveContext: Cannot save context without having computed its hash!");
    return false;
  }
  const auto context_dir {context_base_dir / std::to_string(context.id->value)};
  try {
    if (!fs::exists(context_dir)) {
      fs::create_directories(context_dir);
    }
    drake::yaml::SaveYamlFile(context_dir / CONTEXT_DMD_FILE,
                              context.model_directive);
    drake::yaml::SaveYamlFile(context_dir / CONTEXT_CONSTRAINTS_FILE,
                              context.constraints_adapter);
    if (!context.metadata.empty()) {
      json metadata;
      if (fs::is_regular_file(context_dir / CONTEXT_METADATA_FILE)) {
        // we want to keep the existing metadata, but update it with new values
        json existing_metadata;
        common::utils::LoadJsonFromFile(context_dir / CONTEXT_METADATA_FILE,
                                        existing_metadata);
        metadata = existing_metadata;
      }
      for (const auto& [key, value] : context.metadata.items()) {
        // don't overwrite creation date if already present
        if (key == "creation_date" && metadata.contains(key)) {
          logging::log()->debug(
              "SaveContext: Not overwriting existing creation date");
          continue;
        }
        metadata[key] = value;
      }
      logging::log()->debug("SaveContext: Saving metadata to disk: {}",
                            metadata.dump());
      common::utils::SaveJsonToFile(context_dir / CONTEXT_METADATA_FILE,
                                    metadata);
    }
  } catch (const std::exception& e) {
    logging::log()->error(
        "SaveContext: Failed to save context with ID: {} due to "
        "exception: {}",
        context.id->value, e.what());
    return false;
  }
  context.context_dir = context_dir;
  return true;
}

draco::DracoAdapter MakeDracoAdapterFromContext(
    const std::string& system, const PlanContext& context,
    bool require_parameters,
    std::optional<draco::VisualizerMode> visualizer_mode,
    std::optional<fs::path> options_base_path) {
  draco::DracoAdapter draco_adapter;
  draco_adapter.system = system;
  draco_adapter.context_dir = context.context_dir;
  // construct some context-invariant structures
  const auto options_path {options_base_path.value_or(
      context.context_dir.parent_path().parent_path())};
  const auto visualizer_options_file {options_path / VIZ_OPTIONS_FILE};
  const auto spline_params_path {options_path / SPLINE_PARAMS_FILE};
  const auto dynamic_limits_path {options_path / DYNAMIC_LIMITS_FILE};
  const auto thunder_params_path {options_path / THUNDER_PARAMS_FILE};
  const auto iris_builder_options_path {options_path
                                        / IRIS_BUILDER_OPTIONS_FILE};
  const auto gcs_options_path {options_path / GCS_OPTIONS_FILE};
  const auto spline_params_found {fs::is_regular_file(spline_params_path)};
  const auto dynamic_limits_found {fs::is_regular_file(dynamic_limits_path)};
  const auto thunder_params_found {fs::is_regular_file(thunder_params_path)};
  const auto gcs_options_found {fs::is_regular_file(gcs_options_path)};
  const auto iris_builder_options_found {
      fs::is_regular_file(iris_builder_options_path)};
  if (!(spline_params_found && dynamic_limits_found && thunder_params_found
        && iris_builder_options_found && gcs_options_found)) {
    const auto err_msg {fmt::format(
        "Data directory ({}) does not have all expected files present! spline "
        "params: {}, dynamic limits: {}, thunder params: "
        "{}, iris builder options: {}, gcs options: {}",
        options_path, spline_params_found, dynamic_limits_found,
        thunder_params_found, iris_builder_options_found, gcs_options_found)};
    if (require_parameters) {
      logging::log()->error("MakeDracoAdapterFromContext: {}", err_msg);
      throw std::runtime_error(err_msg);
    }
    logging::log()->warn("MakeDracoAdapterFromContext: {}. Using defaults",
                         err_msg);
  }
  // initialize to defaults
  auto joint_dynamic_limits_map {joint_dynamic_limits_map_t()};
  auto cartesian_dynamic_limits_map {cartesian_dynamic_limits_map_t()};
  try {
    if (fs::is_regular_file(visualizer_options_file)) {
      logging::log()->debug(
          "MakeDracoAdapterFromContext: Loading meshcat params from disk at {}",
          visualizer_options_file);
      service::visualization::VisualizerOptions viz_options =
          drake::yaml::LoadYamlFile<service::visualization::VisualizerOptions>(
              visualizer_options_file);
      draco_adapter.robot_meshcat_params = viz_options.meshcat_params;
    } else {
      logging::log()->warn(
          "MakeDracoAdapterFromContext: No meshcat params file found at {}, "
          "using defaults.",
          visualizer_options_file);
    }
    // Save default splining parameters to disk
    if (!spline_params_found) {
      drake::yaml::SaveYamlFile<TimeOptimalSplineParams>(
          spline_params_path, TimeOptimalSplineParams());
    }
    // Save default thunder parameters to disk
    if (!thunder_params_found) {
      drake::yaml::SaveYamlFile<motion::planning::ompl::ThunderParameters>(
          thunder_params_path, motion::planning::ompl::ThunderParameters());
    }
    // Save default GCS options to disk
    if (!gcs_options_found) {
      drake::yaml::SaveYamlFile<motion::planning::GcsPlannerOptions>(
          gcs_options_path, motion::planning::GcsPlannerOptions());
    }
    // Save default iris builder options to disk
    if (!iris_builder_options_found) {
      drake::yaml::SaveYamlFile<motion::iris::IrisBuilderOptions>(
          iris_builder_options_path, motion::iris::IrisBuilderOptions());
    }
    draco_adapter.time_optimal_spline_params =
        drake::yaml::LoadYamlFile<TimeOptimalSplineParams>(spline_params_path);
    draco_adapter.thunder_parameters =
        drake::yaml::LoadYamlFile<motion::planning::ompl::ThunderParameters>(
            thunder_params_path);
    draco_adapter.iris_builder_options =
        drake::yaml::LoadYamlFile<motion::iris::IrisBuilderOptions>(
            iris_builder_options_path);
    draco_adapter.gcs_planner_options =
        drake::yaml::LoadYamlFile<motion::planning::GcsPlannerOptions>(
            gcs_options_path);
    // Dynamic Limits
    if (dynamic_limits_found) {
      joint_dynamic_limits_map =
          drake::yaml::LoadYamlFile<joint_dynamic_limits_map_t>(
              dynamic_limits_path, "joint_limits");
      cartesian_dynamic_limits_map =
          drake::yaml::LoadYamlFile<cartesian_dynamic_limits_map_t>(
              dynamic_limits_path, "cartesian_limits");
    }

  } catch (const std::exception& e) {
    // rethrow
    throw std::runtime_error(
        fmt::format("Attempted to load parameters from disk, but failed due to "
                    "exception: {}",
                    e.what()));
  }
  // Set options
  draco_adapter.joint_dynamic_limits_map = joint_dynamic_limits_map;
  draco_adapter.cartesian_dynamic_limits_map = cartesian_dynamic_limits_map;

  // Set geo and constraints
  draco_adapter.xml_file = context.urdf_dir / "package.xml";
  draco_adapter.dmd = context.model_directive;
  draco_adapter.constraints_adapter = context.constraints_adapter;

  // Set IRIS regions file to appropriate context path
  auto iris_regions_adapter_file = context.context_dir / IRIS_REGIONS_FILE;
  // Set thunder db path
  draco_adapter.thunder_dat_file = context.context_dir / THUNDER_ROADMAP_FILE;
  // load IRIS regions adapter or create new one
  const auto iris_regions_found {
      fs::is_regular_file(fs::path(iris_regions_adapter_file))};
  auto iris_regions_adapter {
      iris_regions_found ? drake::yaml::LoadYamlFile<IrisRegionsAdapter>(
                               iris_regions_adapter_file)
                         : IrisRegionsAdapter()};
  draco_adapter.iris_regions_adapter = iris_regions_adapter;
  draco_adapter.iris_regions_adapter_file = iris_regions_adapter_file;
  // draco_adapter.make_draco_visualizer = true;
  auto draco_options_path {options_path / DRACO_OPTIONS_FILE};
  bool draco_options_file_found {fs::is_regular_file(draco_options_path)};
  if (!draco_options_file_found) {
    logging::log()->warn(
        "MakeDracoAdapterFromContext: No Draco options file found at {}, "
        "using defaults.",
        draco_options_path);
    draco_adapter.options = draco::DracoOptions();
  } else {
    draco_adapter.options = drake::yaml::LoadYamlFile<draco::DracoOptions>(
        draco_options_path.string());
  }
  if (visualizer_mode) {
    draco_adapter.options.visualizer_options.mode = *visualizer_mode;
  }
  return draco_adapter;
}

const std::vector<PlanContext> LoadAvailableContexts(
    const fs::path& context_base_dir, const ResourceOptions& options) {
  logging::log()->info("LoadAvailableContexts: Loading contexts on disk at: {}",
                       context_base_dir);
  std::vector<PlanContext> contexts;
  int counter {0};
  for (const auto& context_entry : fs::directory_iterator(context_base_dir)) {
    ++counter;
    const auto context_path {context_entry.path()};
    if (!fs::is_directory(context_path)) {
      logging::log()->warn(
          "LoadAvailableContexts: Item: {} in context directory: {} "
          "is invalid!",
          context_path, context_base_dir);
      continue;
    }
    if (context_path.stem() == "visualization") {
      // skip visualization subdirectory
      continue;
    }
    const auto context_dir {context_path.stem().string()};
    const uint64_t context_id {std::stoull(context_dir)};
    const auto base_dir {context_base_dir.parent_path()};
    PlanContext context(context_id);
    if (!LoadContext(context, context_base_dir)) {
      logging::log()->error("LoadAvailableContexts: Failed to load data!");
      continue;
    }
    // Check if DMD includes any of the required names.
    // Note: this is a not a perfect filter and it is hacky. We will filter
    // based on system state rather than DMDs in the future, when system state
    // becomes part of the context:
    // https://github.com/SonyResearch/planning_service_client/pull/154
    if (options.dmd_contains_names.has_value()) {
      // Turn DMD into string
      std::string dmd_string =
          drake::yaml::SaveYamlString(context.model_directive);
      bool let_context_load {true};
      for (const auto& name_substring : options.dmd_contains_names.value()) {
        if (dmd_string.find(name_substring) == std::string::npos) {
          let_context_load = false;
          break;
        }
      }
      if (!let_context_load) {
        logging::log()->info(
            "LoadAvailableContexts: Skipping context ID: {} as its DMD does "
            "not include all the required names: {}",
            context.id->value,
            fmt::join(options.dmd_contains_names.value(), ", "));
        continue;
      }
    }
    contexts.push_back(context);
  }
  logging::log()->info(
      "LoadAvailableContexts: Loaded {}/{} available contexts from {}",
      contexts.size(), counter, context_base_dir);
  return contexts;
}

bool SaveModelFile(const ModelFile& modelfile, const fs::path& model_base_dir,
                   const bool overwrite_existing) {
  // TODO(@davebambrick): add package verification here
  const auto extension {
      "." + common::utils::to_lower(magic_enum::enum_name(modelfile.format))};
  const auto modelfile_path {fs::weakly_canonical(
      model_base_dir / modelfile.parent_path / (modelfile.name + extension))};
  fs::create_directories(modelfile_path.parent_path());

  return common::utils::SaveToFile(modelfile_path, modelfile.content,
                                   overwrite_existing, true);
}

bool SaveMeshFile(const MeshFile& meshfile, const fs::path& model_base_dir,
                  const bool overwrite_existing) {
  const auto meshfile_path {
      fs::weakly_canonical(model_base_dir / meshfile.parent_path
                           / (meshfile.name + meshfile.extension))};
  fs::create_directories(meshfile_path.parent_path());

  return common::utils::SaveToFile(meshfile_path, meshfile.content,
                                   overwrite_existing, true);
}

bool SavePackageToXml(const std::string& package_name,
                      const fs::path& model_base_dir,
                      const bool overwrite_existing) {
  const auto package_xml_path {
      fs::weakly_canonical(model_base_dir / "package.xml")};

  const auto package_xml_string {
      R"(<?xml version="1.0"?>
<package format="2">
  <name>)"
      + package_name + R"(</name>
  <version>0.0.1</version>
</package>
)"};
  return common::utils::SaveToFile(package_xml_path, package_xml_string,
                                   overwrite_existing);
}

bool ContextIsValid(const PlanContext& context) {
  bool valid {true};
  std::string err_msg {""};
  if (context.models.empty()) {
    logging::log()->error(
        "ContextIsValid: Context does not include any models!");
    return false;
  }

  std::set<std::string> context_model_names, context_pkg_names;
  for (const auto& model : context.models) {
    if (model.file.content.empty()) {
      logging::log()->error("ContextIsValid: Model {} contains no data!",
                            model.name);
      return false;
    }
    context_model_names.insert(model.name);
    context_pkg_names.insert(model.file.package_name);
  }
  if (context_pkg_names.size() != 1) {
    err_msg = fmt::format(
        "Multiple model packages are not supported! All models must derive "
        "from the same package. (Packages: {})",
        common::utils::join_strings(context_pkg_names));
    valid = false;
  }
  std::set<std::string> directive_model_names;
  for (const auto& directive : context.model_directive.directives) {
    if (const auto& add_model_opt {directive.add_model};
        add_model_opt.has_value()) {
      directive_model_names.insert(add_model_opt->name);
    }
  }
  if (directive_model_names != context_model_names) {
    err_msg = fmt::format(
        "Model instances of context do not match those defined in the model "
        "directive!\n\tContext:         [{}]\n\tModel directive: [{}]",
        common::utils::join_strings(context_model_names),
        common::utils::join_strings(directive_model_names));
    valid = false;
  }

  if (!context.constraints_adapter.collision_checker.has_value()) {
    err_msg = "Collision checker must not be null!";
    valid = false;
  }
  if (!valid) {
    logging::log()->error("ContextIsValid: {}", err_msg);
  }
  return valid;
}

const std::optional<PlanContextId> RegisterPlanContext(
    const PlanContext& context, const fs::path& urdf_dir) {
  if (!ContextIsValid(context)) {
    return std::nullopt;
  }
  if (!fs::exists(urdf_dir)) {
    fs::create_directories(urdf_dir);
  }
  const auto package_xml_path {urdf_dir / "package.xml"};
  try {
    // 1. search for package xml
    if (!SavePackageToXml(context.models.front().file.package_name, urdf_dir)) {
      logging::log()->error("RegisterPlanContext: Failed to save package.xml!");
      return std::nullopt;
      return std::nullopt;
    }
    // 2. save models to disk if they don't exist
    for (const auto& model : context.models) {
      if (!SaveModelFile(model.file, urdf_dir)) {
        logging::log()->error("RegisterPlanContext: Failed to save model: {}",
                              model.name);
        return std::nullopt;
      }
      for (const auto& meshfile : model.meshes) {
        if (!SaveMeshFile(meshfile, urdf_dir)) {
          logging::log()->error("RegisterPlanContext: Failed to save mesh {}{}",
                                meshfile.name, meshfile.extension);
          return std::nullopt;
        }
      }
    }
  } catch (const std::exception& e) {
    logging::log()->error("RegisterPlanContext:Error while registering! {}",
                          e.what());
    return std::nullopt;
  }
  logging::log()->info("RegisterPlanContext:Parsing model from saved data");
  try {
    // 3. Construct model, constraints, and compute hash
    const auto model {
        motion::RobotModel(package_xml_path, context.model_directive)};
    const auto constraints {
        motion::RobotConstraints(model, context.constraints_adapter)};
    return PlanContextId(std::hash<motion::RobotConstraints> {}(constraints));
  } catch (const std::exception& e) {
    logging::log()->error(
        "RegisterPlanContext:Failed to parse model and constraints from "
        "disk: "
        "{}",
        e.what());
    return std::nullopt;
  }
}

}  // namespace utils
}  // namespace service
