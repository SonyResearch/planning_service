
#pragma once

#include <Eigen/Dense>

#include "planning_service_client/planner/planner_base.h"

namespace planning_service_client {
namespace planner {

using PlanningProblemFactory =
    std::function<std::unique_ptr<PlanningProblemBase>(
        const proto::MotionProblemDefinition&)>;

inline std::string GetProblemCaseName(
    const proto::MotionProblemDefinition& msg) {
  const google::protobuf::Descriptor* descriptor = msg.GetDescriptor();
  const google::protobuf::OneofDescriptor* oneof =
      descriptor->FindOneofByName("problem");

  if (!oneof) return "No `problem` oneof found";

  const google::protobuf::Reflection* reflection = msg.GetReflection();
  const google::protobuf::FieldDescriptor* field =
      reflection->GetOneofFieldDescriptor(msg, oneof);

  if (!field) return "PROBLEM_NOT_SET";

  return std::string(field->name());  // e.g. "start_to_goal_problem"
}

class PlanningProblemRegistry {
 public:
  static void Register(proto::MotionProblemDefinition::ProblemCase type,
                       PlanningProblemFactory factory) {
    GetRegistry()[type] = std::move(factory);
  }

  static std::unique_ptr<PlanningProblemBase> Create(
      const proto::MotionProblemDefinition& msg) {
    auto it = GetRegistry().find(msg.problem_case());
    if (it != GetRegistry().end()) {
      return it->second(msg);
    }
    throw std::runtime_error(
    "Unregistered problem case: " +
    GetProblemCaseName(msg) +
    "\n\nLikely cause: You defined a new PlanningProblem-derived class but forgot to register it.\n"
    "To fix this, add the following in your .cc file:\n"
    "    REGISTER_PROBLEM(YourProblemClass, proto::MotionProblemDefinition::kYourEnumCase, your_proto_field_name);\n"
    "Make sure this appears at global scope.\n");
  }

 private:
  static std::unordered_map<proto::MotionProblemDefinition::ProblemCase,
                            PlanningProblemFactory>&
  GetRegistry() {
    static std::unordered_map<proto::MotionProblemDefinition::ProblemCase,
                              PlanningProblemFactory>
        registry;
    return registry;
  }
};

template <typename ProblemT, typename ProtoT>
bool RegisterPlanningProblem(
    proto::MotionProblemDefinition::ProblemCase case_type,
    const ProtoT& (*get_field)(const proto::MotionProblemDefinition&)) {
  PlanningProblemRegistry::Register(
      case_type, [get_field](const proto::MotionProblemDefinition& msg) {
        return FromProto<ProblemT>(get_field(msg)).Clone();
      });
  return true;
}

#define CLIENT_REGISTER_PLANNING_PROBLEM(ClassName, EnumCase, FieldName) \
                                                                         \
  static const bool _registered_##ClassName = []() {                     \
    PlanningProblemRegistry::Register(                                   \
        EnumCase, [](const proto::MotionProblemDefinition& msg) {        \
          return FromProto<ClassName>(msg.FieldName()).Clone();          \
        });                                                              \
    return true;                                                         \
  }()

}  // namespace planner
}  // namespace planning_service_client
