#include "thunder_planner.h"

#include <drake/geometry/optimization/hyperrectangle.h>
#include <drake/planning/graph_algorithms/max_clique_solver_via_greedy.h>

namespace ot = ::ompl::tools;
namespace ob = ::ompl::base;
namespace og = ::ompl::geometric;

namespace motion {
namespace planning {
namespace ompl {

namespace {
ob::ScopedState<> EigenToScopedState(const Eigen::VectorXd& eigen,
                                     const ob::StateSpacePtr& state_space) {
  // throw if eigen is not the right size
  if (eigen.size() != state_space->getDimension()) {
    throw std::runtime_error("Eigen vector is not the right size");
  }
  ob::ScopedState<> state(state_space);
  std::vector<double> v;
  v.resize(eigen.size());
  Eigen::VectorXd::Map(&v[0], eigen.size()) = eigen;
  state = v;
  return state;
}
}  // namespace

Eigen::VectorXd v_to_e(std::vector<double> v) {
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
}

std::vector<double> e_to_v(Eigen::VectorXd e) {
  std::vector<double> v;
  v.resize(e.size());
  Eigen::VectorXd::Map(&v[0], e.size()) = e;
  return v;
}

ThunderPlanner::ThunderPlanner(const RobotConstraints& robot_constraints,
                               const ThunderParameters& thunder_parameters,
                               const std::string& db_file_path)
    : ThunderPlanner(makeSampleBasedPlanningContext(robot_constraints),
                     thunder_parameters, db_file_path) {}

void ThunderPlanner::SetupThunderPlanner(
    const ThunderParameters& thunder_parameters) {
  logging::log()->info(
      "ThunderPlanner:SetupThunderPlanner: Setting up Thunder planner");
  switch (thunder_parameters.verbosity) {
    case 1:
      ::ompl::msg::setLogLevel(::ompl::msg::LOG_ERROR);
      break;
    case 2:
      ::ompl::msg::setLogLevel(::ompl::msg::LOG_WARN);
      break;
    case 3:
      ::ompl::msg::setLogLevel(::ompl::msg::LOG_INFO);
      break;
    case 4:
      ::ompl::msg::setLogLevel(::ompl::msg::LOG_DEBUG);
      break;
  }
  // get space dimension
  const auto space_dimension {planning_context_->state_space()->getDimension()};
  // sparse_delta is used to control the length of the edges in the roadmap, as
  // well as the visibility of the recall planner. sparse_delta_fraction
  // decreases with dimension to ensure reasonable performance.
  const auto& sparse_delta_fraction {
      thunder_parameters.sparse_delta_fraction_base + 1 / space_dimension};
  setSparseDelta(sparse_delta_fraction);
  setNumParallelPlans(thunder_parameters.num_parallel_plans);
  setRRT();
  getSpaceInformation()->setStateValidityChecker(
      planning_context_->validity_checker());
  getSpaceInformation()->setStateValidityCheckingResolution(
      thunder_parameters.validity_check_resolution);
  getSpaceInformation()->setup();

  ob::OptimizationObjectivePtr optimization_obj {
      std::make_shared<ob::PathLengthOptimizationObjective>(
          getSpaceInformation())};
  if (planning_context_->robot_constraints()
          .constraints_adapter()
          .collision_checker.has_value()
      && planning_context_->robot_constraints()
             .constraints_adapter()
             .collision_checker.value()
             .minimum_value_penalty_params.has_value()) {
    logging::log()->info(
        "ThunderPlanner:SetupThunderPlanner: Setting up optimization objective "
        "with MaxConstraintsClearanceObjective");
    ob::OptimizationObjectivePtr state_cost_obj {
        std::make_shared<MaxConstraintsClearanceObjective>(*planning_context_)};
    optimization_obj = optimization_obj + state_cost_obj;
  }
  // Set the optimization objective
  setOptimizationObjective(optimization_obj);

  // set params of member classes
  getProblemDefinition()->setComputeSolutionCost(
      thunder_parameters.set_compute_solution_cost);
  if (!experience_database()->getSPARSdb()) {
    // Load SPARSdb
    experience_database()->getSPARSdb() =
        std::make_shared<og::SPARSdb>(space_information());
    experience_database()->getSPARSdb()->setProblemDefinition(
        getProblemDefinition());
    experience_database()->getSPARSdb()->setup();
  }
  logging::log()->info(
      "ThunderPlanner:SetupThunderPlanner: Setting up SPARSdb");
  experience_database()->getSPARSdb()->setStretchFactor(getStretchFactor());
  // was 0.05 // vertex visibility range  = maximum_extent * this_fraction
  experience_database()->getSPARSdb()->setSparseDeltaFraction(getSparseDelta());
  experience_database()->getSPARSdb()->setDenseDeltaFraction(getDenseDelta());
  // roadmap_granularity is used to determine the minimum distance enforced
  // between every two nodes in the roadmap. This quantity
  // increases with dimension to ensure the roadmap is not very dense in a way
  // that hinders performance.
  const auto& roadmap_granularity {thunder_parameters.roadmap_granularity_base
                                   + 0.05 * space_dimension};
  getExperienceDB()->getSPARSdb()->setRoadmapGranularity(roadmap_granularity);
  getExperienceDB()->getSPARSdb()->setMigrateRoadmapOnLoad(
      thunder_parameters.migration_on_load);
  getExperienceDB()->getSPARSdb()->setUseCostInRoadmap(
      thunder_parameters.use_cost_in_roadmap);
  getExperienceDB()->getSPARSdb()->setDenseRoadmap(
      thunder_parameters.dense_roadmap);
  getExperienceDB()->getSPARSdb()->setAddEdgeWithCost(
      thunder_parameters.add_edge_with_cost);
  // load experience db
  setFilePath(db_file_path_);
  logging::log()->info(
      "ThunderPlanner:SetupThunderPlanner: Loading experience database from "
      "file: {}",
      getFilePath());
  getExperienceDB()->load(getFilePath());
  // setup SPARSdb
  getExperienceDB()->getSPARSdb()->setup();
  setup();
  GetPlannerData(*planner_data_);
  vertices_confs_.reserve(planner_data_->numVertices());
  for (unsigned int i = 0; i < planner_data_->numVertices(); i++) {
    const ob::State* old_state = planner_data_->getVertex(i).getState();
    ob::State* state = si_->cloneState(old_state);
    std::vector<double> state_vec;
    planning_context_->state_space()->copyToReals(state_vec, state);
    Eigen::VectorXd conf {v_to_e(state_vec)};
    vertices_confs_.push_back(conf);
  }
  logging::log()->info(
      "ThunderPlanner:SetupThunderPlanner: Done setting up "
      "Thunder planner with {} vertices",
      vertices_confs_.size());
}

void ThunderPlanner::AddConfsToRoadmap(
    const std::vector<Eigen::VectorXd>& confs) {
  for (const auto& conf : confs) {
    ob::ScopedState<> state =
        EigenToScopedState(conf, planning_context_->state_space());
    // terminal condition
    ::ompl::base::PlannerTerminationCondition neverTerminate =
        ::ompl::base::plannerNonTerminatingCondition();
    auto num_vertices_before = planner_data_->numVertices();
    // add the state to the sparsdb
    experienceDB_->getSPARSdb()->addStateToRoadmap(neverTerminate, state.get());
    // add to vertices_confs_
    // Create new planner data
    planner_data_ = std::make_unique<ob::PlannerData>(
        planning_context_->space_information());
    GetPlannerData(*planner_data_);
    // if the number of vertices is increased, add the conf to vertices_confs_
    if (planner_data_->numVertices() > num_vertices_before) {
      vertices_confs_.push_back(conf);
      logging::log()->info(
          "ThunderPlanner:AddConfsToRoadmap: Added new configuration to "
          "roadmap, new number of vertices: {}",
          planner_data_->numVertices());
    } else {
      logging::log()->info(
          "ThunderPlanner:AddConfsToRoadmap: Configuration is too close to "
          "existing configuration in roadmap, number of vertices remains: {}",
          planner_data_->numVertices());
    }
  }
  // Save thunder data to database
  SaveThunderDataToDatabase(false);
}

const Eigen::VectorXd& ThunderPlanner::GetVertexConf(int vertex_index) const {
  return vertices_confs_.at(vertex_index);
}

std::vector<int> ThunderPlanner::CalcSortedVertexIndicesNearConf(
    const Eigen::VectorXd& q_start) const {
  std::vector<double> distances(vertices_confs_.size(), 0.0);
  omp_set_num_threads(
      planning_context_->validity_checker()->robot_constraints().num_threads());
#pragma omp parallel for shared(distances)
  for (int i = 0; i < static_cast<int>(vertices_confs_.size()); i++) {
    distances[i] = CalcConfigSpaceDistance(q_start, vertices_confs_[i]);
  }
  // End parallel region
  std::vector<int> indices(vertices_confs_.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), [&distances](int a, int b) {
    return distances[a] < distances[b];
  });
  // Sort the indices based on the distances
  return indices;
}

std::optional<Eigen::VectorXd> ThunderPlanner::CalcNearestValidConf(
    const Eigen::VectorXd& q_invalid,
    const std::vector<drake::multibody::ModelInstanceIndex>&
        fixed_model_instance_indices,
    double offset, double search_step_size,
    std::optional<Eigen::VectorXd> maybe_q_start,
    int num_random_samples) const {
  return CalcNearestValidConf({q_invalid},
                              std::set<drake::multibody::ModelInstanceIndex>(
                                  fixed_model_instance_indices.begin(),
                                  fixed_model_instance_indices.end()),
                              offset, search_step_size, maybe_q_start,
                              num_random_samples);
}

std::optional<Eigen::VectorXd> ThunderPlanner::CalcNearestValidConf(
    const std::vector<Eigen::VectorXd>& q_invalid_vec,
    const std::set<drake::multibody::ModelInstanceIndex>&
        fixed_model_instance_indices,
    double offset, double search_step_size,
    std::optional<Eigen::VectorXd> maybe_q_start,
    int num_random_samples) const {
  // Does not make sense to have multiple invalid configurations without fixed
  // model instance indices
  DRAKE_THROW_UNLESS(!fixed_model_instance_indices.empty()
                     || q_invalid_vec.size() == 1);
  // Let's compute the distance to all vertices in the roadmap, after fixing the
  // fixed_model_instance_indices
  const auto& robot_constraints {
      planning_context_->validity_checker()->robot_constraints()};
  const auto& robot_model = robot_constraints.robot_model();
  std::vector<std::pair<Eigen::VectorXd, double>> conf_distance_vec;
  double min_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < std::ssize(vertices_confs_); i++) {
    // Fix the model instance indices part of it.
    auto projected_to_vertex = robot_model.SetIdleModelsConfigToRef(
        q_invalid_vec[0], vertices_confs_[i], fixed_model_instance_indices);
    double distance =
        CalcConfigSpaceDistance(projected_to_vertex, q_invalid_vec[0]);
    // If the same entry does not exist, add it
    bool entry_exists = false;
    for (const auto& [conf, dist] : conf_distance_vec) {
      if (conf.isApprox(projected_to_vertex, 1e-6)) {
        entry_exists = true;
        break;
      }
    }
    if (!entry_exists) {
      conf_distance_vec.push_back({projected_to_vertex, distance});
      min_distance = std::min(min_distance, distance);
    }
  }
  // Some random search around q_invalid
  if (num_random_samples > 0) {
    drake::RandomGenerator generator(0);
    logging::log()->info(
        "ThunderPlanner:CalcNearestValidConf: Adding {} random "
        "samples within {} around invalid configuration.",
        num_random_samples, min_distance);
    auto lower_limits_reduced = robot_model.holonomic_mapping().Reduce(
        robot_model.plant().GetPositionLowerLimits());
    auto upper_limits_reduced = robot_model.holonomic_mapping().Reduce(
        robot_model.plant().GetPositionUpperLimits());
    auto joint_limits_box = drake::geometry::optimization::Hyperrectangle(
        lower_limits_reduced, upper_limits_reduced);
    int dim = lower_limits_reduced.size();
    DRAKE_DEMAND(dim == q_invalid_vec[0].size());
    auto min_distance_box = drake::geometry::optimization::Hyperrectangle(
        q_invalid_vec[0]
            - Eigen::VectorXd::Constant(dim, min_distance / std::sqrt(dim)),
        q_invalid_vec[0]
            + Eigen::VectorXd::Constant(dim, min_distance / std::sqrt(dim)));
    auto box_intersection_opt =
        joint_limits_box.MaybeGetIntersection(min_distance_box);
    DRAKE_DEMAND(box_intersection_opt.has_value());
    auto box_intersection = box_intersection_opt.value();
    for (int i = 0; i < num_random_samples; i++) {
      // Sample a random direction within the intersection box
      auto q_random = box_intersection.UniformSample(&generator);
      // Fix the model instance indices part of it.
      auto projected_to_random = robot_model.SetIdleModelsConfigToRef(
          q_invalid_vec[0], q_random, fixed_model_instance_indices);
      // For this random confs, though, it is  better to check if they are valid
      if (!robot_constraints.CheckSatisfied(projected_to_random)) {
        continue;
      }
      auto distance =
          CalcConfigSpaceDistance(q_invalid_vec[0], projected_to_random);
      conf_distance_vec.push_back({projected_to_random, distance});
    }
  }
  // Sort the conf_distance_vec based on the distance
  std::sort(conf_distance_vec.begin(), conf_distance_vec.end(),
            [](const std::pair<Eigen::VectorXd, double>& a,
               const std::pair<Eigen::VectorXd, double>& b) {
              return a.second < b.second;
            });
  // Now, let's try to find the best conf that is reachable from q_invalid
  // If no best index was found, return nullopt
  if (conf_distance_vec.empty()) {
    logging::log()->error(
        "ThunderPlanner:CalcNearestValidConf: No valid configuration "
        "found in the roadmap with fixed model instance indices, "
        "returning std::nullopt.");
    return std::nullopt;
  }
  for (int i = 0; i < std::ssize(conf_distance_vec); i++) {
    const auto& [q_suggested, distance] = conf_distance_vec[i];
    logging::log()->debug(
        "\n\nThunderPlanner:CalcNearestValidConf: Trying candidate "
        "configuration {}/{} with and distance {} from {}",
        i + 1, conf_distance_vec.size(), distance,
        i < num_random_samples ? "random samples" : "roadmap");
    std::vector<Eigen::VectorXd> q_result_vec {q_suggested};
    for (int j = 1; j < std::ssize(q_invalid_vec); j++) {
      Eigen::VectorXd q_invalid_j = q_invalid_vec[j];
      robot_model.SetIdleModelsConfigToRef(&q_invalid_j, q_suggested,
                                           fixed_model_instance_indices);
      q_result_vec.push_back(q_invalid_j);
    }
    // Check validity of all q_result_vec here. Because it is lazier, and faster
    // overall. Also uses parallelization internally.
    if (!robot_constraints.CheckSatisfied(q_result_vec)) {
      logging::log()->debug(
          "ThunderPlanner:CalcNearestValidConf: Candidate "
          "configuration {}/{} is not valid, continuing search.",
          i + 1, conf_distance_vec.size());
      continue;
    }
    // Now apply some offset, just to make it more optimal
    const auto& first_q_invalid = q_invalid_vec[0];
    double distance_from_invalid = (q_suggested - first_q_invalid).norm();
    DRAKE_THROW_UNLESS(distance_from_invalid > 1e-6);
    Eigen::VectorXd direction = (q_suggested - first_q_invalid).normalized();
    for (double d = search_step_size; d < distance_from_invalid;
         d += search_step_size) {
      auto q_search = first_q_invalid + d * direction;
      auto q_offset = first_q_invalid + (d + offset) * direction;
      // Let's offset it a bit more, and still check validity
      std::vector<Eigen::VectorXd> q_search_and_offset_vec {q_search, q_offset};
      for (int j = 1; j < std::ssize(q_invalid_vec); j++) {
        auto q_search_j = robot_model.SetIdleModelsConfigToRef(
            q_invalid_vec[j], q_search, fixed_model_instance_indices);
        auto q_offset_j = robot_model.SetIdleModelsConfigToRef(
            q_invalid_vec[j], q_offset, fixed_model_instance_indices);
        q_search_and_offset_vec.push_back(q_search_j);
        q_search_and_offset_vec.push_back(q_offset_j);
      }
      if (robot_constraints.CheckSatisfied(q_search_and_offset_vec)) {
        // If start_conf is provided, check if the edge from start_conf to
        // q_offset is valid
        if (maybe_q_start.has_value()) {
          if (robot_constraints.CheckSatisfiedEdge(maybe_q_start.value(),
                                                   q_offset)) {
            logging::log()->debug(
                "ThunderPlanner:CalcNearestValidConf: Found a valid "
                "edge from start, d:{} + offset:{} = {}, returning it.",
                d, offset, (d + offset));
            return q_offset;
          } else {
            logging::log()->debug(
                "ThunderPlanner:CalcNearestValidConf: The edge from "
                "the provided starting configuration to the candidate "
                "configuration at i {}/{} is not valid, continuing search.",
                i, conf_distance_vec.size());
            break;
          }
        }
        logging::log()->debug(
            "ThunderPlanner:CalcNearestValidConf: Found a valid conf, "
            "d:{} + offset:{} = {}, returning it.",
            d, offset, (d + offset));
        return q_offset;
      }
      logging::log()->debug(
          "ThunderPlanner:CalcNearestValidConf: Candidate "
          "configuration {}/{} at d:{} + offset:{} = {} is not valid, "
          "continuing search.",
          i + 1, conf_distance_vec.size(), d, offset, (d + offset));
    }
  }
  // Nothing worked.
  return std::nullopt;
}

void ThunderPlanner::SaveThunderDataToDatabase(const bool do_post_processing) {
  // do post processing
  logging::log()->info(
      "ThunderPlanner:SaveThunderDataToDatabase: Saving data to: {}",
      getFilePath());
  if (do_post_processing) doPostProcessing();
  logging::log()->info(
      "ThunderPlanner:SaveThunderDataToDatabase: done post processing");
  save();
}

void ThunderPlanner::ReloadAndValidateLocalRoadmap() {
  // set migration on load in SPARSdb
  getExperienceDB()->getSPARSdb()->setMigrateRoadmapOnLoad(true);
  // load the roadmap
  getExperienceDB()->load(getFilePath());
  // reset migratioon on load
  getExperienceDB()->getSPARSdb()->setMigrateRoadmapOnLoad(false);
}

std::optional<std::vector<Eigen::VectorXd>> ThunderPlanner::SolveRecallPlan(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal) {
  logging::log()->debug("ThunderPlanner:SolveRecallPlan: START");
  SetupProblemDefinition(start, goal);
  // termination condition
  const auto recall_timed_ptc {ob::timedPlannerTerminationCondition(
      thunder_parameters_.max_recall_planning_time)};
  // solve
  getRetrieveRepairPlanner().solve(recall_timed_ptc);
  if (haveExactSolutionPath()) {
    auto path {getSolutionPath()};
    std::vector<Eigen::VectorXd> path_vec;
    for (const auto& state : path.getStates()) {
      std::vector<double> state_vec;
      planning_context_->state_space()->copyToReals(state_vec, state);
      Eigen::VectorXd state_eigen =
          Eigen::Map<Eigen::VectorXd>(state_vec.data(), state_vec.size());
      path_vec.push_back(state_eigen);
    }
    const auto success_msg {
        fmt::format("ThunderPlanner:SolveRecallPlan: Found solution from "
                    "recall planner of size: {}",
                    path_vec.size())};
    if (thunder_parameters_.verbosity <= 3) {
      logging::log()->info(success_msg);
    } else {
      logging::log()->debug(success_msg);
    }
    return path_vec;
  }
  logging::log()->warn(
      "ThunderPlanner:SolveRecallPlan: No solution found in recall planning. "
      "Returning nullopt.");
  return std::nullopt;
}

std::optional<std::vector<Eigen::VectorXd>> ThunderPlanner::SolveParallelPlan(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
    const bool save_solution_to_database) {
  logging::log()->debug("ThunderPlanner:SolveParallelPlan: START");
  SetupProblemDefinition(start, goal);
  // termination condition
  const auto scratch_timed_ptc {ob::timedPlannerTerminationCondition(
      thunder_parameters_.max_scratch_planning_time)};
  // solve
  solve(scratch_timed_ptc);
  if (haveExactSolutionPath()) {
    auto path {getSolutionPath()};
    std::vector<Eigen::VectorXd> path_vec;
    for (const auto& state : path.getStates()) {
      std::vector<double> state_vec;
      planning_context_->state_space()->copyToReals(state_vec, state);
      Eigen::VectorXd state_eigen =
          Eigen::Map<Eigen::VectorXd>(state_vec.data(), state_vec.size());
      path_vec.push_back(state_eigen);
    }
    const auto success_msg {
        fmt::format("ThunderPlanner:SolveParallelPlan: Found solution from "
                    "scratch planner of size: {}",
                    path_vec.size())};
    if (thunder_parameters_.verbosity <= 3) {
      logging::log()->info(success_msg);
    } else {
      logging::log()->debug(success_msg);
    }
    if (save_solution_to_database) {
      logging::log()->info(
          "ThunderPlanner:SolveParallelPlan: Saving Thunder data to "
          "database");
      SaveThunderDataToDatabase(true);
    }
    return path_vec;
  }
  logging::log()->error(
      "ThunderPlanner:SolveParallelPlan: No solution found in scratch "
      "planning. Returning std::nullopt.");
  return std::nullopt;
}

const conf_edge_vec_t ThunderPlanner::GetRoadmapEdges() {
  // Add the corresponding edges to the graph
  conf_edge_vec_t edge_vec;
  std::vector<unsigned int> edge_list;
  for (int from_vertex = 0;
       from_vertex < static_cast<int>(planner_data().numVertices());
       ++from_vertex) {
    edge_list.clear();
    // Get the edges
    planner_data().getEdges(from_vertex,
                            edge_list);  // returns num of edges
    // Process edges
    for (unsigned int to_vertex : edge_list) {
      // Get the states
      const ob::State* from_state =
          planner_data().getVertex(from_vertex).getState();
      const ob::State* to_state =
          planner_data().getVertex(to_vertex).getState();

      // Convert to Eigen
      std::vector<double> from_state_vec;
      std::vector<double> to_state_vec;
      planning_context_->state_space()->copyToReals(from_state_vec, from_state);
      planning_context_->state_space()->copyToReals(to_state_vec, to_state);
      Eigen::VectorXd from_conf {v_to_e(from_state_vec)};
      Eigen::VectorXd to_conf {v_to_e(to_state_vec)};
      // Add to edge_vec
      edge_vec.push_back({from_conf, to_conf});
    }
  }
  logging::log()->debug(
      "ThunderPlanner:GetRoadmapEdges: Retrieved {} edges from database",
      edge_vec.size());
  return edge_vec;
}

std::vector<int> ThunderPlanner::GetLargestCliqueContainingEdge(
    unsigned int from_vertex_index, unsigned int to_vertex_index) const {
  // get number of vertices (cast to int)
  unsigned int num_vertices = planner_data().numVertices();
  // throw if the vertex indices are out of range
  if (from_vertex_index >= num_vertices || to_vertex_index >= num_vertices) {
    throw std::runtime_error("Vertex indices out of range");
  }
  auto connected_vertices {
      GetConnectedVertices({from_vertex_index, to_vertex_index})};
  // log the connected vertices one by one
  auto graph_connected_to_edge {
      SparseAdjacencyMatrixOfVertices(connected_vertices)};
  // get the largest clique containing the edge
  drake::planning::graph_algorithms::MaxCliqueSolverViaGreedy max_clique_solver;
  auto max_clique_result {
      max_clique_solver.SolveMaxClique(graph_connected_to_edge)};
  std::vector<int> largest_clique;
  for (int i = 0; i < std::ssize(max_clique_result); ++i) {
    if (max_clique_result[i]) {
      largest_clique.push_back(connected_vertices[i]);
    }
  }
  return largest_clique;
}

std::vector<unsigned int> ThunderPlanner::GetConnectedVertices(
    std::vector<unsigned int> included_vertex_indices) const {
  //  get the indices of the vertices in the planner data that are
  //  simultaneously connected to ALL members of included_vertex_indices
  std::vector<unsigned int> connected_vertex_indices;
  // loop over the included_vertex_indices, and get the connected vertices by
  // calculating the intersection
  for (int i = 0; i < std::ssize(included_vertex_indices); ++i) {
    if (i == 0) {
      planner_data().getEdges(included_vertex_indices[i],
                              connected_vertex_indices);
      // incoming edges too
      std::vector<unsigned int> incoming_vertex_indices;
      planner_data().getIncomingEdges(included_vertex_indices[i],
                                      incoming_vertex_indices);
      connected_vertex_indices.insert(connected_vertex_indices.end(),
                                      incoming_vertex_indices.begin(),
                                      incoming_vertex_indices.end());
    } else {
      std::vector<unsigned int> connected_vertex_indices_temp;
      planner_data().getEdges(included_vertex_indices[i],
                              connected_vertex_indices_temp);
      // incoming edges too
      std::vector<unsigned int> incoming_vertex_indices;
      planner_data().getIncomingEdges(included_vertex_indices[i],
                                      incoming_vertex_indices);
      connected_vertex_indices_temp.insert(connected_vertex_indices_temp.end(),
                                           incoming_vertex_indices.begin(),
                                           incoming_vertex_indices.end());

      // sort the two vectors
      std::sort(connected_vertex_indices.begin(),
                connected_vertex_indices.end());
      std::sort(connected_vertex_indices_temp.begin(),
                connected_vertex_indices_temp.end());
      std::vector<unsigned int> intersection;
      std::set_intersection(connected_vertex_indices.begin(),
                            connected_vertex_indices.end(),
                            connected_vertex_indices_temp.begin(),
                            connected_vertex_indices_temp.end(),
                            std::back_inserter(intersection));
      connected_vertex_indices = intersection;
    }
  }
  return connected_vertex_indices;
}

Eigen::SparseMatrix<bool> ThunderPlanner::SparseAdjacencyMatrixOfVertices(
    const std::vector<unsigned int>& vertex_indices) const {
  // get the number of vertices
  const int num_vertices = std::ssize(vertex_indices);
  // create the sparse matrix
  Eigen::SparseMatrix<bool> adjacency_matrix(num_vertices, num_vertices);
  // fill the adjacency matrix
  for (int i = 0; i < num_vertices; i++) {
    for (int j = i + 1; j < num_vertices; j++) {
      int from_vertex = vertex_indices[i];
      int to_vertex = vertex_indices[j];
      // if an edge exists between the two vertices, add 1 to the corresponding
      // matrix entry

      if (planner_data().edgeExists(from_vertex, to_vertex)
          || planner_data().edgeExists(to_vertex, from_vertex)) {
        adjacency_matrix.insert(i, j) = 1;
        adjacency_matrix.insert(j, i) = 1;
      }
    }
  }
  return adjacency_matrix;
}

double ThunderPlanner::CalcConfigSpaceDistance(
    const Eigen::VectorXd& conf_A, const Eigen::VectorXd& conf_B) const {
  ob::ScopedState<> state_A =
      EigenToScopedState(conf_A, planning_context_->state_space());
  ob::ScopedState<> state_B =
      EigenToScopedState(conf_B, planning_context_->state_space());
  return si_->distance(state_A.get(), state_B.get());
}

ThunderPlanner::ThunderPlanner(std::shared_ptr<SampleBasedPlanningContext> ctx,
                               const ThunderParameters& thunder_parameters,
                               const std::string& db_file_path)
    : ot::Thunder(ctx->space_information()),
      planning_context_(ctx),
      thunder_parameters_ {thunder_parameters},
      db_file_path_ {
          db_file_path.empty()
              ? fmt::format("{}.dat",
                            ctx->robot_constraints().constraints_hash())
              : db_file_path},
      planner_data_ {std::make_unique<ob::PlannerData>(
          planning_context_->space_information())} {
  SetupThunderPlanner(thunder_parameters_);
}

void ThunderPlanner::SetupProblemDefinition(const Eigen::VectorXd& start,
                                            const Eigen::VectorXd& goal) {
  planning_context_->validity_checker()->ValidatePlanningProblemOrThrow(start,
                                                                        goal);

  ob::ScopedState<> start_state =
      EigenToScopedState(start, planning_context_->state_space());
  ob::ScopedState<> goal_state =
      EigenToScopedState(goal, planning_context_->state_space());

  // set the start and goal states
  setStartAndGoalStates(start_state, goal_state);
}

}  // namespace ompl
}  // namespace planning
}  // namespace motion
