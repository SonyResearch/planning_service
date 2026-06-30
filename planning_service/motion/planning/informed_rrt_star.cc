#include "informed_rrt_star.h"
namespace motion {
namespace planning {
namespace ompl {
namespace {

std::vector<Eigen::VectorXd> ConvertPathToEigen(
    og::PathGeometric& path, const ob::StateSpacePtr& state_space) {
  std::vector<Eigen::VectorXd> path_vec;
  for (const auto& state : path.getStates()) {
    std::vector<double> state_vec;
    state_space->copyToReals(state_vec, state);
    Eigen::VectorXd state_eigen =
        Eigen::Map<Eigen::VectorXd>(state_vec.data(), state_vec.size());
    path_vec.push_back(state_eigen);
  }
  return path_vec;
}
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

InformedRRTStarPlanner::InformedRRTStarPlanner(
    const ob::SpaceInformationPtr& si,
    const RobotConstraints& robot_constraints)
    : og::InformedRRTstar(si),
      planning_context_(
          std::make_shared<SampleBasedPlanningContext>(robot_constraints)) {
  SetupPlanner();
}

InformedRRTStarPlanner::InformedRRTStarPlanner(
    const RobotConstraints& robot_constraints)
    : InformedRRTStarPlanner(std::make_shared<ob::SpaceInformation>(
                                 std::make_shared<RobotStateSpace>(
                                     robot_constraints.robot_model())),
                             robot_constraints) {}

void InformedRRTStarPlanner::SetupProblemDefinition(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal) {
  // Validate the planning problem
  planning_context_->validity_checker()->ValidatePlanningProblemOrThrow(start,
                                                                        goal);

  // Convert the Eigen vectors to ScopedState objects
  ob::ScopedState<> start_state =
      EigenToScopedState(start, getSpaceInformation()->getStateSpace());
  ob::ScopedState<> goal_state =
      EigenToScopedState(goal, getSpaceInformation()->getStateSpace());

  pdef_->setStartAndGoalStates(start_state, goal_state);

  logging::log()->info("Problem definition start and goal states set");
  // Clear any past solutions since they no longer correspond to our start and
  // goal states
  pdef_->clearSolutionPaths();
}

void InformedRRTStarPlanner::SetupPlanner() {
  setProblemDefinition(
      std::make_shared<ob::ProblemDefinition>(getSpaceInformation()));
  // Set the state validity checker
  getSpaceInformation()->setStateValidityChecker(
      planning_context_->validity_checker());

  // Set the state validity checking resolution
  getSpaceInformation()->setStateValidityCheckingResolution(0.01);

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
    ob::OptimizationObjectivePtr state_cost_obj {
        std::make_shared<MaxConstraintsClearanceObjective>(*planning_context_)};
    optimization_obj = optimization_obj + state_cost_obj;
  }
  // Set the optimization objective
  getProblemDefinition()->setOptimizationObjective(optimization_obj);

  setup();
}

std::optional<std::vector<Eigen::VectorXd>> InformedRRTStarPlanner::Solve(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
    const double timeout) {
  // Setup the problem definition
  logging::log()->info(
      "Setting up problem definition with start = [{}] and "
      "goal = [{}]",
      start.transpose(), goal.transpose());
  SetupProblemDefinition(start, goal);

  logging::log()->info("Solving InformedRRTStarPlanner");

  const auto timed_ptc {ob::timedPlannerTerminationCondition(
      timeout)};  // termination condition after (timeout) seconds

  // Solve the problem
  solve(timed_ptc);

  // Get the solution path
  ob::PathPtr solution_path = pdef_->getSolutionPath();

  if (solution_path) {
    // Convert the solution path to a vector of Eigen vectors
    return ConvertPathToEigen(*solution_path->as<og::PathGeometric>(),
                              getSpaceInformation()->getStateSpace());
  }

  return std::nullopt;
}

}  // namespace ompl
}  // namespace planning
}  // namespace motion
