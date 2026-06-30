/*
 * Copyright © 2024 Dexai Robotics. All rights reserved.
 */

/// @file informed_rrt_star.h
#pragma once

#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include "planning_service/motion/planning/sample_based_planner.h"
namespace motion {
namespace planning {
namespace ompl {
class InformedRRTStarPlanner : og::InformedRRTstar {
 public:
  // constructor that takes a space information, robot_model, and
  // constraints_adapter
  InformedRRTStarPlanner(const ob::SpaceInformationPtr& si,
                         const RobotConstraints& robot_constraints);

  // constructor that takes a robot_model and a constraints_adapter only, for
  // ease of use by planning service code
  InformedRRTStarPlanner(const RobotConstraints& robot_constraints);

  // setup the problem definition that the planner will solve
  void SetupProblemDefinition(const Eigen::VectorXd& start,
                              const Eigen::VectorXd& goal);

  // setup the member structures in ompl
  void SetupPlanner();

  // timeout in seconds
  std::optional<std::vector<Eigen::VectorXd>> Solve(
      const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
      const double timeout = 30.0);

 private:
  std::shared_ptr<SampleBasedPlanningContext> planning_context_;
};
}  // namespace ompl
}  // namespace planning
}  // namespace motion
