#pragma once
#include <fmt/format.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "planning_service/motion/iris/iris_adapter.h"
#include "planning_service/motion/iris/iris_builder.h"
#include "planning_service/motion/planning/ik_cache.h"
#include "planning_service/motion/planning/ik_planner.h"
#include "planning_service/motion/planning/single_mode_gcs_planner.h"
#include "planning_service/motion/planning/straight_path_planner.h"
#include "planning_service/motion/planning/thunder_planner.h"
#include "planning_service/motion/splining/cubic_spliner.h"
#include "planning_service/motion/splining/time_optimal_spliner.h"
// DracoPlanner includes
#include "planning_service/draco/draco_options.h"
#include "planning_service_client/frame_relative_pose.h"
#include "planning_service_client/planner/motion_plan_result.h"
#include "planning_service_client/planner/plan_options.h"
#include "planning_service_client/planner/planner_base.h"

namespace fs = std::filesystem;
namespace psc = planning_service_client;
namespace draco {

/**
 * @brief Snapshot of the collision options to be passed from Draco to the
 * visualizer when adding visualizations that depend on collision options (e.g.,
 * collision pairs, collision geometries, etc.). Note that this passes the
 * actual data stored with the overrideable structures, and not the structures
 * themselves.
 */
struct CollisionOptionsSnapshot {
  Eigen::MatrixXd padding_matrix;
  Eigen::MatrixXi filter_matrix;
  std::vector<motion::ShapeDescription> collision_shapes;
};

/**
 * @brief A bundle of scoped collision option overrides. For as long as an
 * instance of this struct is kept alive, the specified collision options will
 * be applied to the underlying robot constraints. Upon destruction, the
 * original collision options will be restored.
 */
struct CollisionOptionsScope {
  common::ScopedOverride<Eigen::MatrixXd> padding_matrix_scope;
  common::ScopedOverride<Eigen::MatrixXi> filter_matrix_scope;
  common::ScopedOverride<std::vector<motion::ShapeDescription>>
      collision_shapes_scope;
};

struct DracoAdapter {
  std::string system;
  std::string xml_file;
  drake::multibody::parsing::ModelDirectives dmd;
  std::optional<motion::RobotMeshcatParams> robot_meshcat_params;
  motion::ConstraintsAdapter constraints_adapter;
  motion::iris::IrisRegionsAdapter iris_regions_adapter;
  std::string thunder_dat_file;
  std::string iris_regions_adapter_file;
  motion::iris::IrisBuilderOptions iris_builder_options;
  motion::splining::joint_dynamic_limits_map_t joint_dynamic_limits_map;
  motion::splining::cartesian_dynamic_limits_map_t cartesian_dynamic_limits_map;
  motion::splining::TimeOptimalSplineParams time_optimal_spline_params;
  motion::planning::ompl::ThunderParameters thunder_parameters;
  motion::planning::GcsPlannerOptions gcs_planner_options;
  fs::path context_dir;
  fs::path problems_dir;
  DracoOptions options;
};

class Draco {
 public:
  /**
   * @brief Construct a new Draco object from a DracoAdapter.
   *
   * @param draco_adapter
   */
  Draco(const DracoAdapter& draco_adapter);

  /** Read-only access to the robot model. */
  const motion::RobotModel& robot_model() const {
    return robot_model_;
  }

  /** Read-only access to the robot constraints. */
  const motion::RobotConstraints& robot_constraints() const {
    return *robot_constraints_;
  }

  /**
   * @brief Given a joint configuration, calculate the pose of a frame B
   * relative to a frame A.
   *
   * @param q Joint configuration.
   * @param frame_B_name  Target frame name.
   * @param frame_A_name  Relative frame name.
   * @return const drake::math::RigidTransformd
   */
  drake::math::RigidTransformd CalcRelativePose(
      const motion::system_conf_t sysconf, const std::string& frame_B_name,
      const std::string& frame_A_name = "world") const;

  drake::math::RigidTransformd CalcRelativePose(
      const Eigen::VectorXd& q, const std::string& frame_B_name,
      const std::string& frame_A_name = "world") const;

  const fs::path& context_dir() const {
    return context_dir_;
  }

  /** Read-only access to the DracoOptions. */
  const DracoOptions& options() const {
    return options_;
  }

 protected:
  /**
   * @brief Apply the given collision options in local scope. The original
   * collision options will be restored when the returned ScopedOverride objects
   * are destroyed.
   *
   * @param collision_options Options for collision checking.
   * @return CollisionOptionsScope object that manages the scoped overrides.
   */
  [[nodiscard]] CollisionOptionsScope ApplyScopedCollisionOptions(
      const std::optional<psc::planner::CollisionOptions>& collision_options)
      const;

 private:
  const DracoOptions options_;
  const fs::path context_dir_;
  const motion::RobotModel robot_model_;
  const motion::ConstraintsAdapter constraints_adapter_;

 protected:
  std::shared_ptr<motion::RobotConstraints> robot_constraints_;
};
}  // namespace draco

template <>
struct fmt::formatter<planning_service_client::planner::CollisionPair>
    : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const planning_service_client::planner::CollisionPair& cp,
              FormatContext& ctx) const {
    const auto body_1_repr {cp.body_1().empty() ? "ALL" : cp.body_1()};
    const auto body_2_repr {cp.body_2().empty() ? "ALL" : cp.body_2()};
    return fmt::formatter<std::string_view>::format(
        fmt::format("({}, {})", body_1_repr, body_2_repr), ctx);
  }
};
template <>
struct fmt::formatter<planning_service_client::planner::CollisionPadding>
    : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const planning_service_client::planner::CollisionPadding& cp,
              FormatContext& ctx) const {
    return fmt::formatter<std::string_view>::format(
        fmt::format("({}, {:.3f})", cp.pair(), cp.padding()), ctx);
  }
};
