
#pragma once
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <memory>

#include "planning_service/motion/robot_constraints.h"

namespace ob = ::ompl::base;
namespace motion {
namespace planning {
namespace ompl {
/** \brief A state space representing SE(3) */
class RobotStateSpace : public ob::CompoundStateSpace {
 public:
  /** \brief A state in SE(3): position = (x, y, z), quaternion = (x, y, z, w)
   */
  class StateType : public ob::CompoundStateSpace::StateType {
   public:
    StateType() = default;
  };

  RobotStateSpace(const motion::RobotModel& robot_model);

  ~RobotStateSpace() override = default;

  ob::State* allocState() const override;
  void freeState(ob::State* state) const override;
};
}  // namespace ompl
}  // namespace planning
}  // namespace motion
