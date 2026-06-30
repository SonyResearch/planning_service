#pragma once

#include <drake/common/trajectories/composite_trajectory.h>

#include <filesystem>

#include "planning_service/draco/draco.h"
#include "planning_service/service/types/types.h"

// Filenames
#define NAME_FILE "name.txt"
#define CONTEXT_DMD_FILE "dmd.yaml"
#define CONTEXT_CONSTRAINTS_FILE "constraints.yaml"
#define CONTEXT_METADATA_FILE "metadata.json"
#define SPLINE_PARAMS_FILE "time_optimal_spline_params.yaml"
#define DYNAMIC_LIMITS_FILE "dynamic_limits.yaml"
#define IRIS_BUILDER_OPTIONS_FILE "iris_builder_options.yaml"
#define IRIS_REGIONS_FILE "iris_regions.yaml"
#define THUNDER_PARAMS_FILE "thunder_params.yaml"
#define THUNDER_ROADMAP_FILE "thunder_prm.dat"
#define VIZ_OPTIONS_FILE "visualizer_options.yaml"
#define DRACO_OPTIONS_FILE "draco_options.yaml"
#define IRIS_BUILDER_OPTIONS_FILE "iris_builder_options.yaml"
#define GCS_OPTIONS_FILE "gcs_options.yaml"

namespace service {
namespace visualization {
/**
 * @brief Visualization options not tied directly to the model geometry.
 *
 */
struct VisualizerOptions {
  motion::RobotMeshcatParams meshcat_params {};
  // enable setting joint positions via GUI sliders
  bool enable_sliders {false};
  // enable setting joint positions via external calls
  bool enable_set_configuration {false};

  // If true, frames can be toggled on and off through the web interface;
  // however, this is not a persistent state and will be overridden on refresh.
  bool enable_frame_toggle_web {false};
  // IRIS regions
  bool show_iris_regions {false};
  // PRM
  bool show_prm {false};
  // If true, load a default robot model on startup
  bool default_init {false};
  // Map of default contexts for known regions
  std::map<std::string, uint64_t> default_contexts {};
  // If true, dim inactive robots (requires all meshes to be GLTF)
  bool stream_opacity {false};
  // Show colored disk indicators on joints that are near or at their limits.
  // Color encodes proximity: green (OK) → yellow (warning) → red (at limit).
  bool show_joint_limits {false};
  // Any frame whose name contains the pattern, prefix, or suffix specified here
  // will be ignored in the visualizer
  std::vector<std::string> frame_name_ignore_patterns {};
  std::vector<std::string> frame_name_ignore_prefixes {};
  std::vector<std::string> frame_name_ignore_suffixes {};
  // Initial camera pose. Serializes as [x, y, z] via Drake's Eigen support.
  std::optional<Eigen::Vector3d> default_camera_position;
  std::optional<Eigen::Vector3d> default_camera_target;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(meshcat_params));
    a->Visit(DRAKE_NVP(enable_sliders));
    a->Visit(DRAKE_NVP(enable_set_configuration));
    a->Visit(DRAKE_NVP(enable_frame_toggle_web));
    a->Visit(DRAKE_NVP(show_iris_regions));
    a->Visit(DRAKE_NVP(show_prm));
    a->Visit(DRAKE_NVP(default_init));
    a->Visit(DRAKE_NVP(default_contexts));
    a->Visit(DRAKE_NVP(stream_opacity));
    a->Visit(DRAKE_NVP(show_joint_limits));
    a->Visit(DRAKE_NVP(frame_name_ignore_patterns));
    a->Visit(DRAKE_NVP(frame_name_ignore_prefixes));
    a->Visit(DRAKE_NVP(frame_name_ignore_suffixes));
    a->Visit(DRAKE_NVP(default_camera_position));
    a->Visit(DRAKE_NVP(default_camera_target));
  }
};
}  // namespace visualization
namespace utils {

struct ResourceOptions {
  /** If true, the Draco instance will be constructed as a builder. */
  // ToDo(@ramy): set this to false once we get rid of migration after camera
  // calibration
  bool draco_is_builder {true};

  /** If true, all expected parameter files (for IRIS builder, splining, etc.)
   * must be present in addition to Draco content */
  bool require_parameters {false};

  /** If true, Draco instances will be constructed with unique visualizers per
   * instance. */
  bool make_draco_visualizers {true};

  /** If true, all Draco instances must load successfully from disk or an error
   * is thrown */
  bool require_all_resources {false};

  /** If true, all Draco instances are loaded successfully from disk on
   * initialization */
  bool load_resources_on_init {true};

  /** Whether to load meshcat ports html thread or not. Not needed for unit
   * tests (they slow down the tests), but needed for actual Draco instances. */
  bool make_meshcat_ports_html_thread {true};

  /** Optional override for the HTML port. */
  int html_port {7777};

  /** If true, the CID associated with a given resource will be overwritten if
   * it does not match the assigned CID.
   */
  bool update_resource_ids {false};

  /** If true, perform a dry run, removing resources as soon as they are
   * correctly loaded.
   */
  bool dry_run {false};

  /** If true, load contexts in parallel where possible. */
  bool load_in_parallel {true};

  std::optional<std::vector<std::string>> dmd_contains_names {};

  /** Optional override for the data path root. */
  std::optional<fs::path> data_path_root_override {std::nullopt};

  ResourceOptions() = default;
};

/**
 * @brief Given a manifest file, return a map of unique hashes to Draco
 * pointers.
 *
 * @param context_base_dir path to file
 * @return const draco_map_t
 */
const std::vector<PlanContext> LoadAvailableContexts(
    const fs::path& context_base_dir, const ResourceOptions& options);

/**
 * @brief Make a single Draco adapter from a plan context.
 *
 * @param context Target plan context
 * @param require_parameters when true, requires that a given Draco instance
 * have its corresponding planning artifacts (i.e., IRIS regions and GCS cache)
 * and throws otherwise
 * @param visualizer_mode Visualizer mode to use for the Draco instance.
 * Optional. If not provided, defaults to whatever is set in the options.
 * @return const draco::DracoAdapter
 */
draco::DracoAdapter MakeDracoAdapterFromContext(
    const std::string& system_name, const PlanContext& context,
    bool require_parameters = true,
    std::optional<draco::VisualizerMode> visualizer_mode = std::nullopt,
    std::optional<fs::path> options_base_path = std::nullopt);

/**
 * @brief Save a package.xml to the target directory with the given name.
 *
 * @param package_name Name of package.
 * @param model_base_dir Directory in which package contents are located.
 * @return true if file is saved successfully
 * @return false otherwise
 */
bool SavePackageToXml(const std::string& package_name,
                      const fs::path& model_base_dir,
                      const bool overwrite_existing = true);
/**
 * @brief Save a robot model file.
 *
 * @param model Model file.
 * @param model_base_dir Directory to save model data.
 * @param overwrite_existing If true, overwrite existing model URDF
 * @return true if file is saved successfully
 * @return false otherwise
 */
bool SaveModelFile(const ModelFile& modelfile, const fs::path& model_base_dir,
                   const bool overwrite_existing = true);

/**
 * @brief Save a robot model file.
 *
 * @param mesh Mesh file.
 * @param model_base_dir Directory to save model data.
 * @param overwrite_existing If true, overwrite existing model URDF
 * @return true if file is saved successfully
 * @return false otherwise
 */
bool SaveMeshFile(const MeshFile& meshfile, const fs::path& model_base_dir,
                  const bool overwrite_existing = true);
/**
 * @brief Given a PlanContext, compute its associated unique ID. Will look for
 * URDFs at the path specified by `urdf_dir` or will save the context data if
 * provided.
 *
 * @param context Target plan context
 * @param urdf_dir directory where URDFs are stored
 * @return const std::optional<draco::PlanContextId>
 */
const std::optional<draco::PlanContextId> RegisterPlanContext(
    const PlanContext& context, const fs::path& urdf_dir);
/**
 * @brief Verify whether or not the information in the given plan context is
 * valid and self-consistent. Specifically, checks that:
 *   1. DMD filenames match URDF filenames
 *   2. all Models contain geometry data
 *   3. all Models have the same package name
 *   4. context does not have null collision-checker
 *
 * @param context Target context
 * @return true if all conditions satisfied
 * @return false otherwise
 */
bool ContextIsValid(const PlanContext& context);

bool LoadContext(PlanContext& context, const fs::path& context_base_dir,
                 const std::optional<fs::path>& urdf_base_dir = std::nullopt);

bool SaveContext(PlanContext& context, const fs::path& context_base_dir);

}  // namespace utils
}  // namespace service
