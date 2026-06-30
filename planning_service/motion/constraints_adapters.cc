/// @file constraints_adapter.cc

#include "constraints_adapters.h"

#include "robot_model.h"

namespace motion {

ConstraintsAdapter AddJointLimitToConstraintsAdapter(
    const RobotModel& robot_model,
    const ConstraintsAdapter& constraints_adapter) {
  ConstraintsAdapter constraints_adapter_new {constraints_adapter};
  std::vector<JointPositionsBoxConstraintAdapter> joint_limits_adapter_vec;
  auto upper_limit = robot_model.plant().GetPositionUpperLimits();
  auto lower_limit = robot_model.plant().GetPositionLowerLimits();
  // for fully revolute joints, the lower limit is made -inf and upper limit is
  // made +inf
  for (const int i : robot_model.continuous_revolute_joint_indices()) {
    upper_limit[i] = std::numeric_limits<double>::infinity();
    lower_limit[i] = -std::numeric_limits<double>::infinity();
  }
  const auto system_conf_lower = robot_model.ToSystemConf(lower_limit);
  const auto system_conf_upper = robot_model.ToSystemConf(upper_limit);
  joint_limits_adapter_vec.reserve(system_conf_lower.size());
  DRAKE_DEMAND(system_conf_lower.size() == system_conf_upper.size());
  for (const auto& entry : system_conf_lower) {
    JointPositionsBoxConstraintAdapter joint_limits_adapter;
    joint_limits_adapter.multibody_entity_name = entry.first;
    joint_limits_adapter.lower_bounds = entry.second;
    joint_limits_adapter.upper_bounds = system_conf_upper.at(entry.first);
    joint_limits_adapter_vec.push_back(joint_limits_adapter);
  }
  DRAKE_DEMAND(joint_limits_adapter_vec.size() == system_conf_lower.size());
  if (!constraints_adapter_new.joint_position_box_constraints.has_value()) {
    constraints_adapter_new.joint_position_box_constraints =
        std::vector<JointPositionsBoxConstraintAdapter>();
  }
  for (const auto& entry : joint_limits_adapter_vec) {
    constraints_adapter_new.joint_position_box_constraints.value().push_back(
        entry);
  }
  return constraints_adapter_new;
}

}  // namespace motion
