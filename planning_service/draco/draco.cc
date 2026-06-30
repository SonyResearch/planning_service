/// @file draco.cc

#include "draco.h"

#include <time.h>

#include <random>

#include "client_conversions.h"
#include "planning_service/common/override.h"
#include "planning_service/motion/planning/straight_path_planner.h"
#include "planning_service/motion/splining/internal/path_math.h"

namespace draco {

/**
 * @brief Construct a new Draco object from a DracoAdapter.
 *
 * @param draco_adapter
 */
Draco::Draco(const DracoAdapter& draco_adapter)
    : options_ {draco_adapter.options},
      context_dir_ {draco_adapter.context_dir},
      robot_model_ {
          draco_adapter.robot_meshcat_params.has_value()
                  && draco_adapter.options.visualizer_options.mode
                         == VisualizerMode::kNative
              ? motion::RobotModel(draco_adapter.xml_file, draco_adapter.dmd,
                                   draco_adapter.robot_meshcat_params.value())
              : motion::RobotModel(draco_adapter.xml_file, draco_adapter.dmd)},
      constraints_adapter_ {draco_adapter.constraints_adapter},
      robot_constraints_ {std::make_shared<motion::RobotConstraints>(
          robot_model_, constraints_adapter_)} {}

drake::math::RigidTransformd Draco::CalcRelativePose(
    const motion::system_conf_t sysconf, const std::string& frame_B_name,
    const std::string& frame_A_name) const {
  const auto q = robot_model_.ToGeneralizedPosition(sysconf);
  return CalcRelativePose(q, frame_B_name, frame_A_name);
}

drake::math::RigidTransformd Draco::CalcRelativePose(
    const Eigen::VectorXd& q, const std::string& frame_B_name,
    const std::string& frame_A_name) const {
  const auto& frame_A = robot_model_.GetScopedFrameByName(frame_A_name);
  const auto& frame_B = robot_model_.GetScopedFrameByName(frame_B_name);
  auto X_AB = robot_model_.CalcRelativeTransform(q, frame_A, frame_B);
  return X_AB;
}

CollisionOptionsScope Draco::ApplyScopedCollisionOptions(
    const std::optional<psc::planner::CollisionOptions>& collision_options)
    const {
  common::ScopedOverride padding_matrix_scope(
      robot_constraints_->collision_padding_matrix());
  common::ScopedOverride filter_matrix_scope(
      robot_constraints_->collision_filter_matrix());
  common::ScopedOverride collision_shapes_scope(
      robot_constraints_->added_collision_shapes());
  // Return early if no collision options provided
  if (!collision_options.has_value()) {
    return CollisionOptionsScope {std::move(padding_matrix_scope),
                                  std::move(filter_matrix_scope),
                                  std::move(collision_shapes_scope)};
  }
  // Set up matrix scopes to set collision checker matrices
  // on set/restore.
  padding_matrix_scope.on_event([&]() {
    robot_constraints_->SetActivePaddingMatrix();
  });
  filter_matrix_scope.on_event([&]() {
    robot_constraints_->SetActiveCollisionFilterMatrix();
  });
  // Set up collision geometry to add/remove collision geometries on set/restore
  // respectively.
  collision_shapes_scope.on_set([&]() {
    robot_constraints_->AddCollisionShapes();
  });
  collision_shapes_scope.on_restore([&]() {
    robot_constraints_->RemoveAllAddedCollisionShapes();
  });
  auto collision_shapes = collision_shapes_scope.clone();
  std::map<std::string, int> shape_count;
  for (const auto& shape_in_frame : collision_options->shapes) {
    const auto& type {shape_in_frame.shape().type()};
    // TODO(@davebambrick): Replace with actual shape names.
    const auto name {fmt::format("{}_{}", type, shape_count[type]++)};
    const auto frame_name {
        shape_in_frame.frame().empty() ? "world" : shape_in_frame.frame()};
    const auto X_FG {drake::math::RigidTransformd(
        shape_in_frame.quaternion(), shape_in_frame.translation())};
    const auto& shape {shape_in_frame.shape()};
    collision_shapes.emplace_back(name, frame_name, X_FG,
                                  conversions::ToDrakeShape(shape));
  }
  collision_shapes_scope.set(collision_shapes);

  // paddings
  auto padding_matrix = padding_matrix_scope.clone();
  // Apply group-level before link-level (names with "::") so link-level wins.
  auto paddings_sorted = collision_options->paddings;
  std::stable_sort(
      paddings_sorted.begin(), paddings_sorted.end(),
      [](const auto& a, const auto& b) {
        const auto& a1 = a.pair().body_1();
        const auto& a2 = a.pair().body_2();
        const auto& b1 = b.pair().body_1();
        const auto& b2 = b.pair().body_2();

        const bool a_has_link = (a1.find("::") != std::string::npos)
                                || (a2.find("::") != std::string::npos);
        const bool b_has_link = (b1.find("::") != std::string::npos)
                                || (b2.find("::") != std::string::npos);

        // false (group) first, true (link) last
        return a_has_link < b_has_link;
      });
  if (!paddings_sorted.empty()) {
    for (const auto& padding : collision_options->paddings) {
      robot_constraints_->SetPadding(padding_matrix, padding.padding(),
                                     padding.pair().body_1(),
                                     padding.pair().body_2());
    }
    logging::log()->debug(
        "Draco:ApplyScopedCollisionOptions: Overrode paddings for:\n\t{}",
        fmt::join(paddings_sorted, ",\n\t"));
  }
  padding_matrix_scope.set(padding_matrix);
  // filtering
  auto filter_matrix = filter_matrix_scope.clone();
  for (const auto& pair : collision_options->filtered_pairs) {
    robot_constraints_->SetFiltering(filter_matrix, pair.body_1(),
                                     pair.body_2());
  }
  if (!collision_options->filtered_pairs.empty()) {
    logging::log()->debug(
        "Draco:ApplyScopedCollisionOptions: Filtering collisions for:\n\t{}",
        fmt::join(collision_options->filtered_pairs, ",\n\t"));
  }
  filter_matrix_scope.set(filter_matrix);

  return CollisionOptionsScope {std::move(padding_matrix_scope),
                                std::move(filter_matrix_scope),
                                std::move(collision_shapes_scope)};
}

}  // namespace draco
