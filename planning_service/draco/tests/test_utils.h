/*
 * Copyright © 2025 Sony Research. All rights reserved.
 */
#pragma once

#include "planning_service/draco/draco.h"

namespace draco {
namespace test {

/** DracoAdapterFiles holds the name of the files required to construct a
 * DracoAdapter.
 * @note The names of the files are absolute paths.
 */
struct DracoAdapterFiles {
  /** Xml file for the robot model package. */
  std::string xml_file;

  /** Drake model directives file. */
  std::string dmd_file;

  /** Meshcat parameters file, if available.
   * @note Optional. If not provided, the robot model will be constructed
   * without meshcat.
   */
  std::optional<std::string> meshcat_params_file;

  /** Plan adapter file that describes the global, hard motion constraints. */
  std::string plan_adapter_file;

  /** Thunder roadmap database file. */
  std::string thunder_dat_file;

  /** Iris regions adapter file. */
  std::string iris_regions_adapter_file;

  /** Iris builder options file. */
  std::string iris_builder_options_file;

  /** Dynamic limits file. */
  std::string dynamic_limits_file;

  /** Time optimal spline parameters file. */
  std::string time_optimal_spline_params_file;

  /** Thunder parameters file. */
  std::string thunder_params_file;

  /** GCS planner options file. */
  std::string gcs_planner_options_file;

  /** Context directory path. */
  std::string context_dir_path;

  std::string draco_options_file;
};

DracoAdapter Wallflower();

DracoAdapter DualWallflowers();

DracoAdapter DualPandas();

DracoAdapter Franka();

}  // namespace test
}  // namespace draco
