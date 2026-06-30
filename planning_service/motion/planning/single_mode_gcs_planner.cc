#include "planning_service/motion/planning/single_mode_gcs_planner.h"

#include <drake/geometry/optimization/affine_ball.h>

#include <queue>

namespace motion {
namespace planning {

using vertices_path_t = std::vector<
    const drake::geometry::optimization::GraphOfConvexSets::Vertex*>;

// Class for storing pairwise intersections in Drake way.
// TODO(@sadraddini): This class shall be moved into either public API of Drake,
// or into a separate header file.
class PairwiseIntersections {
 public:
  PairwiseIntersections() = default;

  PairwiseIntersections(const std::vector<std::pair<int, int>>& intersections,
                        const std::vector<Eigen::VectorXd>& offsets)
      : intersections_(intersections), offsets_(offsets) {}

  static PairwiseIntersections CalcPairwiseIntersections(
      const drake::geometry::optimization::ConvexSets& main_regions,
      const std::vector<int>& continuous_revolute_joint_indices) {
    const auto [intersections, offsets] =
        drake::geometry::optimization::ComputePairwiseIntersections(
            main_regions, continuous_revolute_joint_indices);
    return PairwiseIntersections(intersections, offsets);
  }
  // use the signature above to write a function that computes pairwise
  // intersections between two sets of convex sets
  static PairwiseIntersections CalcPairwiseIntersections(
      const drake::geometry::optimization::ConvexSets& main_regions,
      const drake::geometry::optimization::ConvexSets& other_regions,
      const std::vector<int>& continuous_revolute_joint_indices) {
    const auto& [intersections, offsets] =
        drake::geometry::optimization::ComputePairwiseIntersections(
            main_regions, other_regions, continuous_revolute_joint_indices);
    return PairwiseIntersections(intersections, offsets);
  }

  const std::vector<std::pair<int, int>>& intersections() const {
    return intersections_;
  }
  const std::vector<Eigen::VectorXd>& offsets() const {
    return offsets_;
  }

 private:
  std::vector<std::pair<int, int>> intersections_;
  // offset that should be applied to region one to get the intersection with
  // region two
  std::vector<Eigen::VectorXd> offsets_;
};

SingleModeGcsPlanner::SingleModeGcsPlanner(
    const RobotConstraints& robot_constraints,
    const iris::IrisRegionsAdapter& iris_regions_adapter,
    const Eigen::VectorXd& joint_velocity_bound,
    const GcsPlannerOptions& options)
    : robot_constraints_(robot_constraints),
      iris_regions_adapter_(iris_regions_adapter),
      iris_inspector_ {std::make_unique<iris::IrisInspector>(
          robot_constraints_, iris_regions_adapter_)},
      gcs_options_ {options} {
  // Create the graph of configurations
  const int dim {robot_constraints_.robot_model().plant().num_positions()};
  const auto continuous_revolute_joint_indices {
      robot_constraints_.robot_model().continuous_revolute_joint_indices()};
  const auto inverse_joint_velocity_bound {joint_velocity_bound.cwiseInverse()};
  graph_of_configs_ = std::make_unique<internal::GraphOfConfigs>(
      dim, continuous_revolute_joint_indices, inverse_joint_velocity_bound,
      internal::GraphOfConfigs::MetricType::kLInfNorm);
  gcs_traj_opt_ = std::make_unique<
      drake::planning::trajectory_optimization::GcsTrajectoryOptimization>(
      dim, continuous_revolute_joint_indices);

  if (gcs_options_.use_convex_hull_gcs) {
    convex_hull_gcs_ =
        std::make_unique<GraphOfConvexHulls>(iris_regions_adapter_);
  }

  // now add regions to the main gcs
  drake::geometry::optimization::ConvexSets main_regions;
  for (const auto& region : iris_regions_adapter_.regions_vec()) {
    main_regions.push_back(
        drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(
            region.set()));
  }
  const int order {3};
  const double h_min {0.0};
  const double h_max {20.0};
  const auto constraints_name {
      robot_constraints_.constraints_adapter().plan_name};
  const std::string name {fmt::format("main_{}", constraints_name.empty()
                                                     ? "unconstrained"
                                                     : constraints_name)};

  const auto start_time {std::chrono::high_resolution_clock::now()};
  std::vector<std::pair<int, int>> intersections;
  std::vector<Eigen::VectorXd> offsets;
  std::vector<std::vector<Eigen::VectorXd>> intersection_samples;
  DRAKE_DEMAND(iris_regions_adapter_.intersections_vec().has_value());
  const auto& intersections_vec {
      iris_regions_adapter_.intersections_vec().value()};
  logging::log()->info(
      "SingleModeGcsPlanner:Ctor: Added {} intersections to "
      "SingleModeGcsPlanner",
      intersections_vec.size());
  for (const auto& intersection_region : intersections_vec) {
    // poputation of pairwise_intersections from
    // iris_regions_adapter_.intersections_vec()
    intersections.push_back(std::make_pair(intersection_region.index_one(),
                                           intersection_region.index_two()));
    // add the reverse edge
    intersections.push_back(std::make_pair(intersection_region.index_two(),
                                           intersection_region.index_one()));
    // extract offsets
    offsets.push_back(intersection_region.offset());
    offsets.push_back(-intersection_region.offset());
    // extract intersection samples
    intersection_samples.push_back(intersection_region.intersection_samples());
    // update vertex to intersection map
    for (int j = 0; j < std::ssize(intersection_samples.back()); ++j) {
      const std::string name = fmt::format("intersection_{}_{}_SampleNum{}",
                                           intersection_region.index_one(),
                                           intersection_region.index_two(), j);
      const auto* vertex =
          graph_of_configs_->AddVertex(drake::geometry::optimization::Point(
                                           intersection_samples.back().at(j)),
                                       name);
      vertex_to_intersection_[vertex] = std::make_pair(
          intersection_region.index_one(), intersection_region.index_two());
    }
  }
  const PairwiseIntersections pairwise_intersections(intersections, offsets);
  auto& main = gcs_traj_opt_->AddRegions(
      main_regions, pairwise_intersections.intersections(), order, h_min, h_max,
      name, &pairwise_intersections.offsets());
  const auto end_time {std::chrono::high_resolution_clock::now()};
  const auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time)};
  logging::log()->info(
      "SingleModeGcsPlanner:Ctor: Added {} with {} regions to "
      "SingleModeGcsPlanner. Took {} ms",
      name, main.size(), duration.count());
  // Let's add things to Graph
  DRAKE_DEMAND(pairwise_intersections.intersections().size()
               == pairwise_intersections.offsets().size());
  // For those regions that do not have intersections, add the center
  for (int i {0}; i < std::ssize(main_regions); ++i) {
    if (std::find_if(pairwise_intersections.intersections().begin(),
                     pairwise_intersections.intersections().end(),
                     [i](const auto& pair) {
                       return pair.first == i || pair.second == i;
                     })
        != pairwise_intersections.intersections().end()) {
      continue;
    }
    const auto& region = main_regions[i];
    const auto* region_as_polytope =
        dynamic_cast<const drake::geometry::optimization::HPolyhedron*>(
            region.get());
    DRAKE_DEMAND(region_as_polytope != nullptr);
    const auto center = region_as_polytope->ChebyshevCenter();
    const std::string name = fmt::format("center_{}", i);
    const auto* vertex = graph_of_configs_->AddVertex(
        drake::geometry::optimization::Point(center), name);
    vertex_to_intersection_[vertex] = std::make_pair(i, i);
    logging::log()->debug(
        "SingleModeGcsPlanner:Ctor: Added center vertex for region {} because "
        "it does not have any intersections",
        i);
  }
  // Add costs and constraints to the GcsTrajectoryOptimization
  gcs_traj_opt_->AddVelocityBounds(-joint_velocity_bound, joint_velocity_bound);
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(dim, dim);
  for (int i = 0; i < dim; ++i) {
    weight_matrix(i, i) = 1.0 / (joint_velocity_bound(i));
  }
  if (gcs_options_.cost_type == 0) {
    gcs_traj_opt_->AddPathEnergyCost(weight_matrix);
  } else if (gcs_options_.cost_type == 1) {
    gcs_traj_opt_->AddPathLengthCost(weight_matrix);
  } else {
    throw std::runtime_error("Cost type not implemented");
  }
  gcs_traj_opt_->AddPathContinuityConstraints(1);
  // Add edges between the graph_of_configs_ vertices
  add_edges_func_ = [this](auto* vertex) {
    if (vertex->outgoing_edges().size() > 0
        && vertex->incoming_edges().size() > 0) {
      // edges already added
      logging::log()->debug(
          "SingleModeGcsPlanner:Ctor:AddEdgesFunc: Edges already added for "
          "vertex: {}",
          vertex->name());
      return;
    }
    logging::log()->debug(
        "SingleModeGcsPlanner:Ctor:AddEdgesFunc: Adding edges for vertex: {}",
        vertex->name());
    if (vertex_to_intersection_.count(vertex) == 0) {
      return;
    }
    const auto [index_a, index_b] = vertex_to_intersection_.at(vertex);
    for (auto* other_vertex : graph_of_configs_->Vertices()) {
      if (vertex == other_vertex) {
        // no self-loops
        continue;
      }
      // Add a vertex if they share a region
      if (vertex_to_intersection_.count(other_vertex) == 0) {
        continue;
      }
      const auto& [other_index_a, other_index_b] =
          vertex_to_intersection_.at(other_vertex);
      if (index_a == other_index_a || index_a == other_index_b
          || index_b == other_index_a || index_b == other_index_b) {
        graph_of_configs_->AddEdge(vertex, other_vertex);
      }
    }
    logging::log()->debug(
        "SingleModeGcsPlanner:Ctor:AddEdgesFunc: Added {} edges for vertex: {}",
        vertex->outgoing_edges().size(), vertex->name());
  };
  if (!gcs_options_.lazy_gcc_edges) {
    logging::log()->info(
        "SingleModeGcsPlanner:Ctor: Using eager edge addition strategy. Wait "
        "....");
    // Add edges for all vertices in prior
    for (auto* vertex : graph_of_configs_->Vertices()) {
      add_edges_func_(vertex);
    }
    // set the add_edges_func_ to nullptr so that no more edges are added
    add_edges_func_ = {};
    logging::log()->info(
        "SingleModeGcsPlanner:Ctor: Added {} vertices and {} edges to the "
        "graph_of_configs_",
        graph_of_configs_->Vertices().size(),
        graph_of_configs_->Edges().size());
  } else {
    logging::log()->info(
        "SingleModeGcsPlanner:Ctor: Using lazy edge addition strategy");
  }
  const auto postprocess_duration {
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::high_resolution_clock::now() - end_time)};
  logging::log()->info(
      "SingleModeGcsPlanner:Ctor: Postprocessing GCS took {} ms",
      postprocess_duration.count());
}

std::optional<drake::trajectories::CompositeTrajectory<double>>
SingleModeGcsPlanner::CalcOptimalPath(const Eigen::VectorXd& q1,
                                      const Eigen::VectorXd& q2) {
  logging::log()->info(
      "SingleModeGcsPlanner:CalcOptimalPath: Computing shortest path beween "
      "nodes:\nq1: [{}]\nq2: [{}]",
      q1.transpose(), q2.transpose());
  TerminalNode start_terminal = AddTerminal(q1, TerminalType::kStart);
  TerminalNode end_terminal = AddTerminal(q2, TerminalType::kEnd);
  const auto start_gcc = start_terminal.q_gcc;
  const auto end_gcc = end_terminal.q_gcc;
  const auto start_found {start_gcc != nullptr};
  const auto end_found {end_gcc != nullptr};
  // print the gcs and gcc graphs
  if (!(start_found && end_found)) {
    logging::log()->error(
        "SingleModeGcsPlanner:CalcOptimalPath: Failed to add point(s) to "
        "graph: {}{}",
        start_found ? "" : fmt::format("\n\tstart: [{}]", q1.transpose()),
        end_found ? "" : fmt::format("\n\tend: [{}]", q2.transpose()));
    return std::nullopt;
  }
  const auto [_, gcc_path] =
      graph_of_configs_->CalcShortestPath(start_gcc, end_gcc, &add_edges_func_);
  // ToDo(@sadra): The code up to here is the same as CalcOptimalPathCHulls
  // and FastEstimatePathLength. Make it a reusable function.
  if (gcc_path.empty()) {
    logging::log()->error(
        "SingleModeGcsPlanner:CalcOptimalPath: Failed to find a path!");
    return std::nullopt;
  }
  auto gcs_path = ConvertToGcsVertices(gcc_path, start_terminal, end_terminal);
  for (const auto* vertex : gcs_path) {
    logging::log()->info("SingleModeGcsPlanner:CalcOptimalPath: GCS vertex: {}",
                         vertex->name());
  }
  // now solve for trajectory
  logging::log()->debug(
      "SingleModeGcsPlanner:CalcOptimalPath: Solving for trajectory");
  const auto [traj, result] = gcs_traj_opt_->SolveConvexRestriction(gcs_path);
  logging::log()->debug(
      "SingleModeGcsPlanner:CalcOptimalPath: Trajectory solved with result: {}",
      result.get_solution_result());
  if (!result.is_success()) {
    logging::log()->error(
        "SingleModeGcsPlanner:CalcOptimalPath: Failed to solve for "
        "trajectory");
  }
  // remove the terminal nodes
  RemoveTerminalNode(&start_terminal);
  RemoveTerminalNode(&end_terminal);
  return traj;
}

std::optional<drake::trajectories::CompositeTrajectory<double>>
SingleModeGcsPlanner::CalcOptimalPathCHulls(const Eigen::VectorXd& q1,
                                            const Eigen::VectorXd& q2) {
  logging::log()->info(
      "SingleModeGcsPlanner:CalcOptimalPath: Computing shortest path beween "
      "nodes:\nq1: [{}]\nq2: [{}]",
      q1.transpose(), q2.transpose());
  TerminalNode start_terminal = AddTerminal(q1, TerminalType::kStart);
  TerminalNode end_terminal = AddTerminal(q2, TerminalType::kEnd);
  const auto start_gcc = start_terminal.q_gcc;
  const auto end_gcc = end_terminal.q_gcc;
  const auto start_found {start_gcc != nullptr};
  const auto end_found {end_gcc != nullptr};
  // print the gcs and gcc graphs
  if (!(start_found && end_found)) {
    logging::log()->error(
        "SingleModeGcsPlanner:CalcOptimalPath: Failed to add point(s) to "
        "graph: {}{}",
        start_found ? "" : fmt::format("\n\tstart: [{}]", q1.transpose()),
        end_found ? "" : fmt::format("\n\tend: [{}]", q2.transpose()));
    return std::nullopt;
  }
  const auto [_, gcc_path] =
      graph_of_configs_->CalcShortestPath(start_gcc, end_gcc, &add_edges_func_);
  if (gcc_path.empty()) {
    logging::log()->error(
        "SingleModeGcsPlanner:CalcOptimalPath: Failed to find a path!");
    return std::nullopt;
  }

  int start_point = 1;
  int end_point = 1;

  drake::geometry::optimization::ConvexSets start_term =
      drake::geometry::optimization::MakeConvexSets(
          drake::geometry::optimization::Point(q1));
  drake::geometry::optimization::ConvexSets end_term =
      drake::geometry::optimization::MakeConvexSets(
          drake::geometry::optimization::Point(q2));

  if (start_terminal.visible_gcs_subgraph != nullptr) {
    start_point++;
    start_term.push_back(
        drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(
            *start_terminal.visible_gcs_set));
  }

  if (end_terminal.visible_gcs_subgraph != nullptr) {
    end_point++;
    end_term.push_back(
        drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(
            *end_terminal.visible_gcs_set));
  }

  vertices_path_t gcc_path_slice(gcc_path.begin() + start_point,
                                 gcc_path.end() - end_point);
  auto gcs_path = ConvertToCHullVertices(gcc_path_slice);

  // now solve for trajectory
  const auto traj =
      convex_hull_gcs_->SolveConvexRestriction(start_term, end_term, gcs_path);
  // remove the terminal nodes
  RemoveTerminalNode(&start_terminal);
  RemoveTerminalNode(&end_terminal);
  return traj;
}

std::optional<double> SingleModeGcsPlanner::FastEstimatePathLength(
    const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) {
  logging::log()->debug(
      "SingleModeGcsPlanner:FastEstimatePathLength: Computing shortest path "
      "beween nodes:\nq1:[{}]\nq2: [{}]",
      q1.transpose(), q2.transpose());
  TerminalNode start_terminal = AddTerminal(q1, TerminalType::kStart);
  TerminalNode end_terminal = AddTerminal(q2, TerminalType::kEnd);
  const auto start_gcc = start_terminal.q_gcc;
  const auto end_gcc = end_terminal.q_gcc;
  const auto start_found {start_gcc != nullptr};
  const auto end_found {end_gcc != nullptr};
  // print the gcs and gcc graphs
  if (!(start_found && end_found)) {
    logging::log()->error(
        "SingleModeGcsPlanner:FastEstimatePathLength: Failed to add point(s) "
        "to "
        "graph: {}{}",
        start_found ? "" : fmt::format("\n\tstart: [{}]", q1.transpose()),
        end_found ? "" : fmt::format("\n\tend: [{}]", q2.transpose()));
    return std::nullopt;
  }
  const auto [cost, gcc_path] =
      graph_of_configs_->CalcShortestPath(start_gcc, end_gcc, &add_edges_func_);
  if (gcc_path.empty()) {
    logging::log()->error(
        "SingleModeGcsPlanner:FastEstimatePathLength: Failed to find a path!");
    return std::nullopt;
  }
  // remove the terminal nodes
  RemoveTerminalNode(&start_terminal);
  RemoveTerminalNode(&end_terminal);
  return cost;
}

namespace {
std::map<const drake::geometry::optimization::GraphOfConvexSets::Vertex*, int>
CalcHopsFromGoal(
    const vertices_path_t& important_nodes,
    const drake::geometry::optimization::GraphOfConvexSets::Vertex* goal) {
  std::map<const drake::geometry::optimization::GraphOfConvexSets::Vertex*, int>
      vertex_distance_from_goal;
  std::queue<const drake::geometry::optimization::GraphOfConvexSets::Vertex*>
      queue;
  queue.push(goal);
  vertex_distance_from_goal[goal] = 0;
  // print the important nodes
  while (!queue.empty()) {
    const auto* current_vertex = queue.front();
    queue.pop();
    const auto& incoming_edges = current_vertex->incoming_edges();
    for (const auto* edge : incoming_edges) {
      const auto* next_vertex = &(edge->u());
      // skip if next_vertex is not in important_nodes
      if (std::find(important_nodes.begin(), important_nodes.end(), next_vertex)
          == important_nodes.end()) {
        continue;
      }
      if (vertex_distance_from_goal.count(next_vertex) == 0) {
        vertex_distance_from_goal[next_vertex] =
            vertex_distance_from_goal[current_vertex] + 1;
        queue.push(next_vertex);
      }
    }
  }
  return vertex_distance_from_goal;
}
}  // namespace

void SingleModeGcsPlanner::RemoveTerminalNode(TerminalNode* terminal_node) {
  if (terminal_node->q_subgraph != nullptr) {
    gcs_traj_opt_->RemoveSubgraph(*terminal_node->q_subgraph);
  }
  if (terminal_node->visible_gcs_subgraph != nullptr) {
    gcs_traj_opt_->RemoveSubgraph(*terminal_node->visible_gcs_subgraph);
  }
  if (terminal_node->q_gcc != nullptr) {
    graph_of_configs_->RemoveVertex(terminal_node->q_gcc);
  }
  if (terminal_node->q_visible_gcc != nullptr) {
    graph_of_configs_->RemoveVertex(terminal_node->q_visible_gcc);
  }
}

vertices_path_t SingleModeGcsPlanner::ConvertToGcsVertices(
    const vertices_path_t& gcc_path,
    const SingleModeGcsPlanner::TerminalNode& start_terminal,
    const SingleModeGcsPlanner::TerminalNode& end_terminal) const {
  auto existing_subgraphs = gcs_traj_opt_->GetSubgraphs();
  auto* main_subgraph = existing_subgraphs.front();
  vertices_path_t all_possible_participating_nodes;
  for (const auto* gcc_vertex : gcc_path) {
    if (vertex_to_intersection_.count(gcc_vertex) > 0) {
      const auto [index_a, index_b] = vertex_to_intersection_.at(gcc_vertex);
      const auto* gcs_vertex_a = main_subgraph->Vertices()[index_a];
      const auto* gcs_vertex_b = main_subgraph->Vertices()[index_b];
      if (std::find(all_possible_participating_nodes.begin(),
                    all_possible_participating_nodes.end(), gcs_vertex_a)
          == all_possible_participating_nodes.end()) {
        all_possible_participating_nodes.push_back(gcs_vertex_a);
      }
      if (std::find(all_possible_participating_nodes.begin(),
                    all_possible_participating_nodes.end(), gcs_vertex_b)
          == all_possible_participating_nodes.end()) {
        all_possible_participating_nodes.push_back(gcs_vertex_b);
      }
    }
  }
  const auto* goal_gcs_vertex = end_terminal.q_subgraph->Vertices().front();
  if (start_terminal.visible_gcs_subgraph != nullptr) {
    all_possible_participating_nodes.push_back(
        start_terminal.visible_gcs_subgraph->Vertices().front());
  }
  if (end_terminal.visible_gcs_subgraph != nullptr) {
    all_possible_participating_nodes.push_back(
        end_terminal.visible_gcs_subgraph->Vertices().front());
  }
  const auto hops_from_goal =
      CalcHopsFromGoal(all_possible_participating_nodes, goal_gcs_vertex);
  // let's check the start terminal
  DRAKE_DEMAND(start_terminal.q_subgraph != nullptr);
  DRAKE_DEMAND(start_terminal.q_subgraph->size() == 1);
  const auto* start_gcs_vertex = start_terminal.q_subgraph->Vertices().front();
  vertices_path_t gcs_path;
  gcs_path.push_back(start_gcs_vertex);
  while (true) {
    const auto* current_vertex = gcs_path.back();
    // if we are at the goal, we are done
    if (current_vertex == goal_gcs_vertex) {
      return gcs_path;
    }
    const auto& outgoing_edges = current_vertex->outgoing_edges();
    int min_hops = std::numeric_limits<int>::max();
    int index_min_hops = -1;
    for (int i = 0; i < std::ssize(outgoing_edges); ++i) {
      const auto* edge = outgoing_edges[i];
      const auto* next_vertex = &(edge->v());
      if (hops_from_goal.count(next_vertex) > 0
          && hops_from_goal.at(next_vertex) < min_hops) {
        min_hops = hops_from_goal.at(next_vertex);
        index_min_hops = i;
      }
    }
    DRAKE_DEMAND(index_min_hops != -1);
    gcs_path.push_back(&(outgoing_edges[index_min_hops]->v()));
  }
  return gcs_path;
}

std::vector<int> SingleModeGcsPlanner::ConvertToCHullVertices(
    const vertices_path_t& gcc_path_slice) const {
  auto existing_subgraphs = gcs_traj_opt_->GetSubgraphs();
  std::vector<int> chull_verts;
  const auto& [first_i, first_j] =
      vertex_to_intersection_.at(gcc_path_slice.at(0));
  if (gcc_path_slice.size() == 1 && first_i == first_j) {
    chull_verts.push_back(first_j);
  } else if (gcc_path_slice.size() == 1) {
    chull_verts.push_back(std::min(first_i, first_j));
    chull_verts.push_back(std::max(first_i, first_j));
  } else {
    const auto& [second_i, second_j] =
        vertex_to_intersection_.at(gcc_path_slice.at(1));
    if (first_i == second_i || first_i == second_j) {
      chull_verts.push_back(first_j);
      chull_verts.push_back(first_i);
    } else {
      chull_verts.push_back(first_i);
      chull_verts.push_back(first_j);
    }
    bool first = true;
    for (const auto* gcc_vertex : gcc_path_slice) {
      if (first) {
        first = false;
        continue;
      }
      const auto [index_a, index_b] = vertex_to_intersection_.at(gcc_vertex);
      int last_element = chull_verts.back();

      if (index_a == last_element) {
        chull_verts.push_back(index_b);
      } else {
        chull_verts.push_back(index_a);
      }
    }
  }

  std::vector<int> gcs_reg_indices;

  for (const auto reg_ind : chull_verts) {
    gcs_reg_indices.push_back(
        iris_regions_adapter_.regions_vec().at(reg_ind).index());
  }

  return gcs_reg_indices;
}

namespace {
// TEMPORARY: until MakeAffineBallFromLineSegment gets solved on the drake
// side - which requires a change in the drake codebase and will be slow.
// UPDATE (Sep 2025): The drake side would also benefit from the changes below.
drake::geometry::optimization::AffineBall CustomMakeAffineBallFromLineSegment(
    const Eigen::Ref<const Eigen::VectorXd>& x_1,
    const Eigen::Ref<const Eigen::VectorXd>& x_2, double epsilon) {
  DRAKE_THROW_UNLESS(x_1.size() == x_2.size());
  DRAKE_THROW_UNLESS(epsilon > 0.0);
  const double length = (x_1 - x_2).norm();
  const double kTolerance = 1e-9;
  if (length < kTolerance) {
    throw std::runtime_error(fmt::format(
        "AffineBall:MakeAffineBallFromLineSegment: x_1 and x_2 are the same "
        "point (distance: {} < tolerance: {}).",
        length, kTolerance));
  }
  const int dim = x_1.size();
  const Eigen::VectorXd center = 0.5 * (x_1 + x_2);
  const Eigen::VectorXd r0 = (x_1 - x_2) / length;
  // Build a matrix whose first column is r0 and the rest are identity basis
  Eigen::MatrixXd M(dim, dim);
  M.col(0) = r0;
  M.rightCols(dim - 1) = Eigen::MatrixXd::Identity(dim, dim).rightCols(dim - 1);
  // Orthonormalize
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(M);
  Eigen::MatrixXd Q = qr.householderQ();

  Eigen::MatrixXd R = Q.leftCols(dim);
  Eigen::MatrixXd scale_matrix = epsilon * Eigen::MatrixXd::Identity(dim, dim);
  scale_matrix(0, 0) = length / 2.0;
  return drake::geometry::optimization::AffineBall(R * scale_matrix, center);
}
}  // namespace

std::unique_ptr<drake::geometry::optimization::ConvexSet>
SingleModeGcsPlanner::CreateConnectingSet(
    const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
    const ConnectingSetType& connecting_set_type, const double epsilon) {
  const int n = q1.size();
  DRAKE_THROW_UNLESS(n == q2.size());
  if (connecting_set_type == ConnectingSetType::kVPolytope) {
    Eigen::MatrixXd connecting_vertices(n, 2);
    connecting_vertices.col(0) = q1;
    connecting_vertices.col(1) = q2;
    return std::make_unique<drake::geometry::optimization::VPolytope>(
        connecting_vertices);
  } else if (connecting_set_type == ConnectingSetType::kNarrowBox) {
    const double length = (q2 - q1).norm();
    auto affine_ball = CustomMakeAffineBallFromLineSegment(q1, q2, 1.0);
    auto R = affine_ball.B();
    R.col(0) = R.col(0) / length * 2;
    DRAKE_DEMAND(std::abs(R.col(0).norm() - 1.0) < 1e-6);
    const auto& center = affine_ball.center();
    Eigen::MatrixXd A(2 * n, n);
    Eigen::VectorXd b(2 * n);
    A.block(0, 0, n, n) = R.transpose();
    A.block(n, 0, n, n) = -R.transpose();
    b.segment(0, n) =
        R.transpose() * center + Eigen::VectorXd::Constant(n, epsilon);
    b.segment(n, n) =
        -R.transpose() * center + Eigen::VectorXd::Constant(n, epsilon);
    b(0) = b(0) + length / 2.0;
    b(n) = b(n) + length / 2.0;
    return std::make_unique<drake::geometry::optimization::HPolyhedron>(A, b);
  } else {
    throw std::runtime_error("Connecting set type not implemented");
  }
}

SingleModeGcsPlanner::TerminalNode SingleModeGcsPlanner::AddTerminal(
    const Eigen::VectorXd& q, const TerminalType& terminal_type) {
  logging::log()->debug(
      "SingleModeGcsPlanner:AddTerminal: Adding terminal of type: {} at point: "
      "[{}]",
      terminal_type == TerminalType::kStart ? "start" : "end", q.transpose());
  auto result = iris_inspector_->EvalConfigAgainstIrisRegions(q);
  if (result.containing_regions_indices.size() > 0) {
    logging::log()->debug(
        "SingleModeGcsPlanner:AddTerminal: Point is covered by volume");
    return DoAddConfig(result.containing_regions_indices, q, terminal_type);
  } else if (result.visible_region_index.has_value()) {
    logging::log()->debug("SingleModeGcsPlanner:AddTerminal: Point is visible");
    return DoAddConnectingSet(
        q,
        std::make_pair(result.visible_region_index.value(),
                       result.visible_point.value()),
        terminal_type);
  }
  logging::log()->error(
      "SingleModeGcsPlanner:AddTerminal: Point [{}] is not covered by volume "
      "and not visible, so we cannot add it to the graph",
      q.transpose());
  return TerminalNode();
}

SingleModeGcsPlanner::TerminalNode SingleModeGcsPlanner::DoAddConfig(
    const std::vector<int>& region_indices, const Eigen::VectorXd& q,
    const TerminalType& terminal_type) {
  auto existing_subgraphs = gcs_traj_opt_->GetSubgraphs();
  auto* main_subgraph = existing_subgraphs.front();
  const std::string name = fmt::format(
      "q_{}", terminal_type == TerminalType::kStart ? "start" : "end");
  const auto q_point = drake::geometry::optimization::Point(q);
  auto& q_subgraph = gcs_traj_opt_->AddRegions(
      drake::geometry::optimization::MakeConvexSets(q_point), 0, 0, 20, name);
  auto* vertex_q = graph_of_configs_->AddVertex(q_point, name);
  int num_gcs_edges_before = gcs_traj_opt_->graph_of_convex_sets().num_edges();
  if (terminal_type == TerminalType::kStart) {
    gcs_traj_opt_->AddEdges(q_subgraph, *main_subgraph);
  } else {
    gcs_traj_opt_->AddEdges(*main_subgraph, q_subgraph);
  }
  int num_gcs_edges_after = gcs_traj_opt_->graph_of_convex_sets().num_edges();
  if (num_gcs_edges_after == num_gcs_edges_before) {
    auto msg = fmt::format(
        "SingleModeGcsPlanner:DoAddConfig: Failed to connect the terminal "
        "node to the main GCS. This is a numerical issue. The point [{}] is in "
        "the region(s) [{}], but failed to connect to any of the main GCS "
        "regions in {}",
        q.transpose(), fmt::join(region_indices, ", "),
        terminal_type == TerminalType::kStart ? "outgoing" : "incoming");
    logging::log()->error(msg);
    throw std::runtime_error(msg);
  }
  // connect the vertex_q to all vertices that are in one of the region indices
  for (auto* v : graph_of_configs_->Vertices()) {
    if (vertex_to_intersection_.count(v) == 0) {
      continue;
    }
    const auto& [intersecting_idx_0, intersecting_idx_1] =
        vertex_to_intersection_.at(v);
    for (const auto& region_index : region_indices) {
      if (intersecting_idx_0 == region_index
          || intersecting_idx_1 == region_index) {
        if (terminal_type == TerminalType::kStart) {
          graph_of_configs_->AddEdge(vertex_q, v);
        } else {
          graph_of_configs_->AddEdge(v, vertex_q);
        }
        break;
      }
    }
  }
  TerminalNode terminal_node {.q_subgraph = &q_subgraph,
                              .visible_gcs_subgraph = nullptr,
                              .visible_gcs_set = nullptr,
                              .q_gcc = vertex_q,
                              .q_visible_gcc = nullptr,
                              .visible_region_index = std::nullopt};
  return terminal_node;
}

SingleModeGcsPlanner::TerminalNode SingleModeGcsPlanner::DoAddConnectingSet(
    const Eigen::VectorXd& q,
    const std::pair<int, Eigen::VectorXd>& visible_result,
    const TerminalType& terminal_type) {
  auto existing_subgraphs = gcs_traj_opt_->GetSubgraphs();
  auto* main_subgraph = existing_subgraphs.front();
  // Get the data
  const auto [visible_region_index, visible_q] = visible_result;
  auto point_q = drake::geometry::optimization::Point(q);
  auto point_visible_q = drake::geometry::optimization::Point(visible_q);
  // First, add the points to the graph_of_configs_
  const auto q_name = fmt::format(
      "q_{}", terminal_type == TerminalType::kStart ? "start" : "end");
  const auto visible_q_name =
      fmt::format("point_visible_{}", visible_region_index,
                  terminal_type == TerminalType::kStart ? "start" : "end");
  auto* vertex_q = graph_of_configs_->AddVertex(point_q, q_name);
  auto* visible_vertex_q =
      graph_of_configs_->AddVertex(point_visible_q, visible_q_name);
  if (terminal_type == TerminalType::kStart) {
    graph_of_configs_->AddEdge(vertex_q, visible_vertex_q);
  } else {
    graph_of_configs_->AddEdge(visible_vertex_q, vertex_q);
  }
  // now add the edge between the visible vertex and all vertices that are in
  // the same iris region
  bool found_vertices = false;
  for (auto* v : graph_of_configs_->Vertices()) {
    if (vertex_to_intersection_.count(v) == 0) {
      continue;
    }
    const auto& intersecting_regions = vertex_to_intersection_.at(v);
    if (intersecting_regions.first == visible_region_index
        || intersecting_regions.second == visible_region_index) {
      if (terminal_type == TerminalType::kStart) {
        graph_of_configs_->AddEdge(visible_vertex_q, v);
      } else {
        graph_of_configs_->AddEdge(v, visible_vertex_q);
      }
      found_vertices = true;
    }
  }
  DRAKE_DEMAND(found_vertices);  // otherwise, we have a bug
  auto connecting_set = CreateConnectingSet(
      q, visible_q,
      static_cast<ConnectingSetType>(gcs_options_.connecting_set_type),
      gcs_options_.connection_set_epsilon);
  auto& q_subgraph = gcs_traj_opt_->AddRegions(
      drake::geometry::optimization::MakeConvexSets(point_q), 0, 0, 20, q_name);
  auto& connecting_subgraph = gcs_traj_opt_->AddRegions(
      drake::geometry::optimization::MakeConvexSets(*connecting_set),
      main_subgraph->order(), 0, 20, visible_q_name);
  // ToDo(@Sadra): We should save the offsets here rather than recomputing them
  // using ComputePairwiseIntersections
  int num_gcs_edges_before = gcs_traj_opt_->graph_of_convex_sets().num_edges();
  if (terminal_type == TerminalType::kStart) {
    auto pairs = std::vector<std::pair<int, int>> {std::make_pair(0, 0)};
    gcs_traj_opt_->AddEdges(q_subgraph, connecting_subgraph, nullptr, &pairs);
    logging::log()->debug(
        "SingleModeGcsPlanner:DoAddConnectingSet: Adding connecting to main");
    pairs = std::vector<std::pair<int, int>> {
        std::make_pair(0, visible_region_index)};
    auto [_, offsets] =
        drake::geometry::optimization::ComputePairwiseIntersections(
            drake::geometry::optimization::MakeConvexSets(*connecting_set),
            drake::geometry::optimization::MakeConvexSets(
                *main_subgraph->regions()[visible_region_index]),
            robot_constraints_.robot_model()
                .continuous_revolute_joint_indices(),
            false);
    gcs_traj_opt_->AddEdges(connecting_subgraph, *main_subgraph, nullptr,
                            &pairs, &offsets);
    logging::log()->debug(
        "SingleModeGcsPlanner:DoAddConnectingSet: Added connecting to main");
  } else {
    auto pairs = std::vector<std::pair<int, int>> {std::make_pair(0, 0)};
    gcs_traj_opt_->AddEdges(connecting_subgraph, q_subgraph, nullptr, &pairs);
    pairs = std::vector<std::pair<int, int>> {
        std::make_pair(visible_region_index, 0)};
    logging::log()->debug(
        "SingleModeGcsPlanner:DoAddConnectingSet: Adding main to connecting");
    auto [_, offsets] =
        drake::geometry::optimization::ComputePairwiseIntersections(
            drake::geometry::optimization::MakeConvexSets(
                *main_subgraph->regions()[visible_region_index]),
            drake::geometry::optimization::MakeConvexSets(*connecting_set),
            robot_constraints_.robot_model()
                .continuous_revolute_joint_indices(),
            false);
    gcs_traj_opt_->AddEdges(*main_subgraph, connecting_subgraph, nullptr,
                            &pairs, &offsets);
    logging::log()->debug(
        "SingleModeGcsPlanner:DoAddConnectingSet: Added main to connecting");
  }
  int num_gcs_edges_after = gcs_traj_opt_->graph_of_convex_sets().num_edges();
  if (num_gcs_edges_after == num_gcs_edges_before) {
    auto msg = fmt::format(
        "SingleModeGcsPlanner:DoAddConnectingSet: Failed to connect the "
        "terminal node to the main GCS. This is a numerical issue. The point "
        "[{}] is visible from region {}, but failed to connect to the main GCS "
        "region {}",
        q.transpose(), visible_region_index,
        terminal_type == TerminalType::kStart ? "outgoing" : "incoming");
    logging::log()->error(msg);
    throw std::runtime_error(msg);
  }
  // We are done!
  TerminalNode terminal_node {.q_subgraph = &q_subgraph,
                              .visible_gcs_subgraph = &connecting_subgraph,
                              .visible_gcs_set = std::move(connecting_set),
                              .q_gcc = vertex_q,
                              .q_visible_gcc = visible_vertex_q,
                              .visible_region_index = visible_region_index};
  return terminal_node;
}

}  // namespace planning
}  // namespace motion
