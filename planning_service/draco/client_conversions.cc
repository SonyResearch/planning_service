#include "client_conversions.h"

#include <drake/common/overloaded.h>
namespace draco {
namespace conversions {

Eigen::VectorXd ToGeneralizedPosition(
    const motion::RobotModel& robot_model,
    const planning_service_client::SystemConf& system_conf,
    const ToGeneralizedBehavior& behavior,
    std::optional<planning_service_client::SystemConf> sysconf_ref_opt) {
  const auto& plant = robot_model.plant();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(plant.num_positions());
  for (int i {0}; i < plant.num_model_instances(); ++i) {
    const auto idx {drake::multibody::ModelInstanceIndex(i)};
    if (plant.num_positions(idx) == 0) {
      continue;
    }
    const auto model_name = plant.GetModelInstanceName(idx);
    if (system_conf.has_key(model_name)) {
      auto q_idx = system_conf.at(model_name).q();
      if (q_idx.size() != plant.num_positions(idx)) {
        auto msg = fmt::format(
            "ToGeneralizedPosition: Size mismatch for robot instance: {}. "
            "Expected {}, got {}.",
            model_name, plant.num_positions(idx), q_idx.size());
        throw msg;
      }
      plant.SetPositionsInArray(idx, q_idx, &q);
    } else if (behavior == ToGeneralizedBehavior::kThrowOnMissing) {
      throw std::runtime_error(
          "ToGeneralizedPosition: Missing configuration for robot "
          "instance: "
          + model_name);
    } else if (behavior
               == ToGeneralizedBehavior::kCompleteFromReferenceOnMissing) {
      if (!sysconf_ref_opt.has_value()
          || !sysconf_ref_opt.value().has_key(model_name)) {
        throw std::runtime_error(
            "ToGeneralizedPosition: Missing reference configuration for "
            "completing missing configurations.");
      }
      auto q_idx = sysconf_ref_opt.value().at(model_name).q();
      plant.SetPositionsInArray(idx, q_idx, &q);
      logging::log()->debug(
          "ToGeneralizedPosition: Completing missing configuration for "
          "robot instance: {}",
          model_name);
    }
  }
  return q;
}

drake::trajectories::PathParameterizedTrajectory<double>
ToPathParameterizedTrajectory(
    const motion::splining::TimeOptimalSpliner& time_optimal_spliner,
    const planning_service_client::SystemTimedTrajectory& sys_timed_trajectory,
    const ToGeneralizedBehavior& behavior) {
  std::map<std::string,
           std::pair<drake::trajectories::PiecewisePolynomial<double>,
                     drake::trajectories::PiecewisePolynomial<double>>>
      name_to_parameterized_trajectory;
  const auto& plant = time_optimal_spliner.plant();
  for (int i {0}; i < plant.num_model_instances(); ++i) {
    const auto idx {drake::multibody::ModelInstanceIndex(i)};
    if (plant.num_positions(idx) > 0) {
      const auto model_name = plant.GetModelInstanceName(idx);
      if (!sys_timed_trajectory.has_key(model_name)) {
        if (behavior == ToGeneralizedBehavior::kThrowOnMissing) {
          throw std::runtime_error(
              "ToPathParameterizedTrajectory: Missing trajectory for robot "
              "instance: "
              + model_name);
        }
        logging::log()->warn(
            "ToPathParameterizedTrajectory: Missing trajectory for robot "
            "instance: {}",
            model_name);
        continue;
      }
      auto traj = sys_timed_trajectory.at(model_name);
      name_to_parameterized_trajectory[model_name] =
          std::make_pair(ClientPiecewisePolynomialToDrake(traj.path()),
                         ClientPiecewisePolynomialToDrake(traj.time_scaling()));
    }
  }
  return time_optimal_spliner.ConvertToPathParameterizedTrajectory(
      name_to_parameterized_trajectory);
}

planning_service_client::SystemTimedTrajectory ToSystemTimedTrajectory(
    const motion::splining::TimeOptimalSpliner& time_optimal_spliner,
    const drake::trajectories::PathParameterizedTrajectory<double>& ppt) {
  planning_service_client::SystemTimedTrajectory sys_timed_trajectory;
  auto map_traj_pairs = time_optimal_spliner.SlicePerEntities(ppt);
  for (auto& [model_name, traj_pair] : map_traj_pairs) {
    auto& [path, time_parameterization] = traj_pair;
    sys_timed_trajectory[model_name] = planning_service_client::TimedTrajectory(
        DrakePiecewisePolynomialToClient(path),
        DrakePiecewisePolynomialToClient(time_parameterization));
  }
  return sys_timed_trajectory;
}

drake::trajectories::PiecewisePolynomial<double>
ClientPiecewisePolynomialToDrake(
    const planning_service_client::PiecewisePolynomial& poly) {
  std::vector<Eigen::MatrixXd> coefficients_vec = poly.coefficients_vec();
  std::vector<double> breaks = poly.breaks();
  std::vector<
      drake::trajectories::PiecewisePolynomial<double>::PolynomialMatrix>
      poly_matrix_vec;
  for (int i = 0; i < std::ssize(coefficients_vec); ++i) {
    const auto& coefficients = coefficients_vec[i];
    Eigen::MatrixX<drake::Polynomiald> poly_matrix(coefficients.rows(), 1);
    for (int j = 0; j < coefficients.rows(); ++j) {
      poly_matrix(j, 0) = drake::Polynomial<double>(coefficients.row(j));
    }
    poly_matrix_vec.push_back(poly_matrix);
  }
  return drake::trajectories::PiecewisePolynomial<double>(poly_matrix_vec,
                                                          breaks);
}

planning_service_client::PiecewisePolynomial DrakePiecewisePolynomialToClient(
    const drake::trajectories::PiecewisePolynomial<double>& poly) {
  planning_service_client::PiecewisePolynomial client_poly;
  std::vector<Eigen::MatrixXd> coefficients_vec;
  for (int i = 0; i < poly.get_number_of_segments(); ++i) {
    const auto poly_matrix = poly.getPolynomialMatrix(i);
    DRAKE_DEMAND(poly_matrix.cols() == 1);
    int degree = poly_matrix(0, 0).GetDegree();
    Eigen::MatrixXd coefficients(poly_matrix.rows(), degree + 1);
    for (int j = 0; j < poly_matrix.rows(); ++j) {
      Eigen::VectorXd coeffs = poly_matrix(j, 0).GetCoefficients();
      DRAKE_DEMAND(coeffs.rows() <= degree + 1);
      coefficients.row(j) = coeffs.transpose();
    }
    coefficients_vec.push_back(coefficients);
  }
  return planning_service_client::PiecewisePolynomial(coefficients_vec,
                                                      poly.get_segment_times());
}

planning_service_client::SystemConf DracoToClientSystemConf(
    const system_conf_t& sysconf) {
  planning_service_client::SystemConf sysconf_client;
  for (const auto& [key, value] : sysconf) {
    sysconf_client[key] = value;
  }
  return sysconf_client;
}

system_conf_t ClientToDracoSystemConf(
    const planning_service_client::SystemConf& sysconf_client) {
  system_conf_t sysconf;
  for (const auto& [key, value] : sysconf_client) {
    sysconf[key] = value.q();
  }
  return sysconf;
}

FrameRelativePoses ToDracoFrameRelativePoses(
    const motion::RobotModel& robot_model,
    const std::vector<planning_service_client::FrameRelativePose>& frp_vec) {
  FrameRelativePoses frps;
  for (const auto& frp : frp_vec) {
    const auto& frame_A_str = frp.frame_A();
    const auto& frame_B_str = frp.frame_B();
    const auto& frame_A = robot_model.GetScopedFrameByName(frame_A_str);
    const auto& frame_B = robot_model.GetScopedFrameByName(frame_B_str);
    drake::math::RigidTransformd X_AB(frp.X_AB_quaternion(),
                                      frp.X_AB_translation());
    frps.emplace_back(&frame_A, &frame_B, X_AB);
  }
  return frps;
}
std::shared_ptr<drake::geometry::Shape> ToDrakeShape(
    const planning_service_client::Shape& shape) {
  return shape.Visit<std::shared_ptr<drake::geometry::Shape>>(overloaded {
      [](const planning_service_client::Sphere& s) {
        return std::make_shared<drake::geometry::Sphere>(s.radius());
      },
      [](const planning_service_client::Cylinder& c) {
        return std::make_shared<drake::geometry::Cylinder>(c.radius(),
                                                           c.height());
      },
      [](const planning_service_client::Capsule& c) {
        return std::make_shared<drake::geometry::Capsule>(c.radius(),
                                                          c.height());
      },
      [](const planning_service_client::Box& b) {
        return std::make_shared<drake::geometry::Box>(b.width(), b.depth(),
                                                      b.height());
      },
      [](const auto& s) -> std::shared_ptr<drake::geometry::Shape> {
        throw std::runtime_error(
            fmt::format("ToDrakeShape: Unsupported shape type: {}", s.type()));
      },
  });
}
}  // namespace conversions
}  // namespace draco
