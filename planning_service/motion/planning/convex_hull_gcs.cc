/// @file convex_hull_gcs.cc

#include "convex_hull_gcs.h"

namespace motion {
namespace planning {

using ConvexHull = drake::geometry::optimization::ConvexHull;
using Subgraph = drake::planning::trajectory_optimization::
    GcsTrajectoryOptimization::Subgraph;

// Constructor implementation
GraphOfConvexHulls::GraphOfConvexHulls(
    const iris::IrisRegionsAdapter& iris_regions_adapter)
    : iris_regions_adapter_(iris_regions_adapter) {
  if (!iris_regions_adapter_.intersections_vec()) {
    logging::log()->error(
        "Pre-condition for convex hulls is that intersections are constructed");
  }

  auto& intersections = iris_regions_adapter_.intersections_vec().value();

  if (iris_regions_adapter_.intersections_vec().value().empty()) {
    logging::log()->info(
        "Skipping CHull construction because there are no intersections");
    return;
  }

  for (auto reg1 : intersections) {
    for (auto reg2 : intersections) {
      int mid_ind = -1;  // Initialize to an invalid value
      int start_ind = -1;
      int end_ind = -1;

      if (reg1.index_one() == reg2.index_one()) {
        mid_ind = reg1.index_one();
        start_ind = std::min(reg1.index_two(), reg2.index_two());
        end_ind = std::max(reg1.index_two(), reg2.index_two());
      } else if (reg1.index_two() == reg2.index_one()) {
        mid_ind = reg1.index_two();
        start_ind = std::min(reg1.index_one(), reg2.index_two());
        end_ind = std::max(reg1.index_one(), reg2.index_two());
      } else if (reg1.index_one() == reg2.index_two()) {
        mid_ind = reg1.index_one();
        start_ind = std::min(reg1.index_two(), reg2.index_one());
        end_ind = std::max(reg1.index_two(), reg2.index_one());
      } else if (reg1.index_two() == reg2.index_two()) {
        mid_ind = reg1.index_two();
        start_ind = std::min(reg1.index_one(), reg2.index_one());
        end_ind = std::max(reg1.index_one(), reg2.index_one());
      }

      if (mid_ind == -1) {
        continue;
      }

      const auto tuple_key = std::make_tuple(start_ind, mid_ind, end_ind);

      auto insertion_result = key_set_.insert(tuple_key);

      if (!insertion_result.second) {
        continue;
      }

      int main_ind = std::size(convex_hulls_);
      drake::geometry::optimization::ConvexSets convex_hull_sets;
      // ToDo(@sadra): save the "simplified" intersection and avoid these
      // unnecessary copies.
      const auto& set1_1 =
          iris_regions_adapter_.regions_vec().at(reg1.index_one());
      const auto& set1_2 =
          iris_regions_adapter_.regions_vec().at(reg1.index_two());
      const auto& set2_1 =
          iris_regions_adapter_.regions_vec().at(reg2.index_one());
      const auto& set2_2 =
          iris_regions_adapter_.regions_vec().at(reg2.index_two());
      const auto hpoly1 = set1_1.set().Intersection(set1_2.set());
      const auto hpoly2 = set2_1.set().Intersection(set2_2.set());
      convex_hull_sets.push_back(
          drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(
              hpoly1));
      convex_hull_sets.push_back(
          drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(
              hpoly2));
      const auto convex_hull = ConvexHull(convex_hull_sets);

      index_mapping_.insert({tuple_key, main_ind});

      convex_hulls_.push_back(
          ConvexHullUnit(convex_hull, start_ind, mid_ind, end_ind, main_ind));
    }
  }

  ConstructCHullAdj();

  int dim =
      iris_regions_adapter_.regions_vec().front().set().ambient_dimension();

  gcs_traj_opt_ = std::make_unique<
      drake::planning::trajectory_optimization::GcsTrajectoryOptimization>(dim);

  drake::geometry::optimization::ConvexSets cHulls;

  // Iterate over the convex_hulls_ vector and extract ConvexHull objects
  for (const auto& wrap : convex_hulls_) {
    cHulls.push_back(
        drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(
            wrap.convex_hull()));  // Add the ConvexHull to the new vector
  }

  const int order {3};
  const double h_min {0.0};
  const double h_max {20.0};

  main_regions_ = &gcs_traj_opt_->AddRegions(cHulls, adj_list_, order, h_min,
                                             h_max, "main_regions");

  gcs_traj_opt_->AddPathEnergyCost();
  gcs_traj_opt_->AddPathContinuityConstraints(1);
}

void GraphOfConvexHulls::UpdateSubgraphMap() {
  std::vector<std::string> subgraph_names;
  for (Subgraph* subgraph : gcs_traj_opt_->GetSubgraphs()) {
    subgraph_names.push_back(subgraph->name());
  }

  subgraph_to_index_.clear();

  for (int i = 0; i < std::ssize(subgraph_names); ++i) {
    subgraph_to_index_[subgraph_names.at(i)] = i;
  }
}

// Method to solve convex restriction
std::optional<drake::trajectories::CompositeTrajectory<double>>
GraphOfConvexHulls::SolveConvexRestriction(
    const drake::geometry::optimization::ConvexSets& q1,
    const drake::geometry::optimization::ConvexSets& q2,
    std::vector<int> desired_sets) {
  if (desired_sets.size() < 2) {
    logging::log()->warn(
        "Since the points are contained by the same region, no convex hulls "
        "are needed for this planning");
    std::vector<const drake::geometry::optimization::GraphOfConvexSets::Vertex*>
        desired_sets_gcs;
    auto gcs_traj_opt_temp =
        drake::planning::trajectory_optimization::GcsTrajectoryOptimization(
            iris_regions_adapter_.regions_vec()
                .front()
                .set()
                .ambient_dimension());

    auto* main_region = &gcs_traj_opt_temp.AddRegions(
        drake::geometry::optimization::MakeConvexSets(
            iris_regions_adapter_.regions_vec().at(desired_sets.front()).set()),
        3);

    auto start_point = &gcs_traj_opt_temp.AddRegions(
        drake::geometry::optimization::MakeConvexSets(*q1[0]), 0);
    gcs_traj_opt_temp.AddEdges(*start_point, *main_region);
    desired_sets_gcs.push_back(start_point->Vertices().front());

    if (q1.size() > 1) {
      auto start_region = &gcs_traj_opt_temp.AddRegions(
          drake::geometry::optimization::MakeConvexSets(*q1[1]), 3);
      gcs_traj_opt_temp.AddEdges(*start_point, *start_region);
      gcs_traj_opt_temp.AddEdges(*start_region, *main_region);
      desired_sets_gcs.push_back(start_region->Vertices().front());
    }

    desired_sets_gcs.push_back(main_region->Vertices().front());

    auto end_point = &gcs_traj_opt_temp.AddRegions(
        drake::geometry::optimization::MakeConvexSets(*q2[0]), 0);
    gcs_traj_opt_temp.AddEdges(*main_region, *end_point);

    desired_sets_gcs.push_back(end_point->Vertices().front());
    if (q2.size() > 1) {
      auto end_region = &gcs_traj_opt_temp.AddRegions(
          drake::geometry::optimization::MakeConvexSets(*q2[1]), 3);
      gcs_traj_opt_temp.AddEdges(*end_region, *end_point);
      gcs_traj_opt_temp.AddEdges(*main_region, *end_region);
      desired_sets_gcs.push_back(end_region->Vertices().front());
    }

    const auto [traj, result] =
        gcs_traj_opt_temp.SolveConvexRestriction(desired_sets_gcs);

    if (!result.is_success()) {
      logging::log()->error(
          "SingleModeGcsPlanner:CalcOptimalPath: Failed to solve for "
          "trajectory");
    }
    return traj;
  } else if (desired_sets.size() < 3) {
    std::pair<int, int> startInter = {
        std::min(desired_sets.at(0), desired_sets.at(1)),
        std::max(desired_sets.at(0), desired_sets.at(1))};
    std::pair<int, int> endInter = {
        std::min(desired_sets.back(), desired_sets[desired_sets.size() - 2]),
        std::max(desired_sets.back(), desired_sets[desired_sets.size() - 2])};
    AddTerminalNode(q1, startInter, true);
    AddTerminalNode(q2, endInter, false);
  } else {
    auto startCHull = std::make_tuple(desired_sets.at(0), desired_sets.at(1),
                                      desired_sets.at(2));
    auto endCHull = std::make_tuple(desired_sets.back(),
                                    desired_sets[desired_sets.size() - 2],
                                    desired_sets[desired_sets.size() - 3]);
    AddTerminalNode(q1, startCHull, true);
    AddTerminalNode(q2, endCHull, false);
  }
  UpdateSubgraphMap();
  auto main_regions =
      gcs_traj_opt_->GetSubgraphs().at(subgraph_to_index_["main_regions"]);
  auto main_reg_vertices = main_regions->Vertices();
  std::vector<const drake::geometry::optimization::GraphOfConvexSets::Vertex*>
      desired_cHulls;
  auto start_point =
      gcs_traj_opt_->GetSubgraphs().at(subgraph_to_index_["start_point"]);
  desired_cHulls.push_back(start_point->Vertices().at(0));
  auto start_region =
      gcs_traj_opt_->GetSubgraphs().at(subgraph_to_index_["start_region"]);
  auto start_reg_verts = start_region->Vertices();
  desired_cHulls.insert(desired_cHulls.end(), start_reg_verts.begin(),
                        start_reg_verts.end());
  for (int i = 1; i < (std::ssize(desired_sets) - 1); ++i) {
    int mid_ind = desired_sets.at(i);
    int start_ind = std::min((desired_sets.at(i - 1)), desired_sets.at(i + 1));
    int end_ind = std::max((desired_sets.at(i - 1)), desired_sets.at(i + 1));
    int cHull_index =
        index_mapping_.at(std::make_tuple(start_ind, mid_ind, end_ind));
    desired_cHulls.push_back(main_reg_vertices.at(cHull_index));
  }
  auto end_region =
      gcs_traj_opt_->GetSubgraphs().at(subgraph_to_index_["end_region"]);
  auto end_reg_verts = end_region->Vertices();
  desired_cHulls.insert(desired_cHulls.end(), end_reg_verts.begin(),
                        end_reg_verts.end());
  auto end_point =
      gcs_traj_opt_->GetSubgraphs().at(subgraph_to_index_["end_point"]);
  auto end_pt_verts = end_point->Vertices();
  desired_cHulls.insert(desired_cHulls.end(), end_pt_verts.begin(),
                        end_pt_verts.end());
  const auto [traj, result] =
      gcs_traj_opt_->SolveConvexRestriction(desired_cHulls);

  if (!result.is_success()) {
    logging::log()->error(
        "SingleModeGcsPlanner:CalcOptimalPath: Failed to solve for "
        "trajectory");
  }
  RemTerminalNode();
  return traj;
}

void GraphOfConvexHulls::ConstructCHullAdj() {
  int numChulls = convex_hulls_.size();
  adj_matrix_.resize(numChulls, std::vector<int>(numChulls, 0));

  for (int i = 0; i < numChulls; i++) {
    auto cHull1 = convex_hulls_.at(i);
    std::pair<int, int> cHull1_inter1 = {
        std::min(cHull1.start_index(), cHull1.mid_index()),
        std::max(cHull1.start_index(), cHull1.mid_index())};
    std::pair<int, int> cHull1_inter2 = {
        std::min(cHull1.end_index(), cHull1.mid_index()),
        std::max(cHull1.end_index(), cHull1.mid_index())};
    for (int j = 0; j < numChulls; j++) {
      if (i >= j) {
        continue;
      }
      auto cHull2 = convex_hulls_.at(j);
      std::pair<int, int> cHull2_inter1 = {
          std::min(cHull2.start_index(), cHull2.mid_index()),
          std::max(cHull2.start_index(), cHull2.mid_index())};
      std::pair<int, int> cHull2_inter2 = {
          std::min(cHull2.end_index(), cHull2.mid_index()),
          std::max(cHull2.end_index(), cHull2.mid_index())};

      if (cHull1_inter1 == cHull2_inter1 || cHull1_inter1 == cHull2_inter2
          || cHull1_inter2 == cHull2_inter1 || cHull1_inter2 == cHull2_inter2) {
        // adjacency found!

        adj_matrix_[i][j] = 1;
        adj_matrix_[j][i] = 1;
        adj_list_.push_back({i, j});
        adj_list_.push_back({j, i});
      }
    }
  }
}

// Method to add terminal node
void GraphOfConvexHulls::AddTerminalNode(
    drake::geometry::optimization::ConvexSets q,
    const std::pair<int, int> desired_intersection, const bool start) {
  auto& intersections = iris_regions_adapter_.intersections_vec().value();

  drake::geometry::optimization::HPolyhedron desired_set;

  for (auto reg : intersections) {
    if (std::make_pair(reg.index_one(), reg.index_two())
        == desired_intersection) {
      const auto& set_1 =
          iris_regions_adapter_.regions_vec().at(reg.index_one());
      const auto& set_2 =
          iris_regions_adapter_.regions_vec().at(reg.index_two());
      const auto intersect = set_1.set().Intersection(set_2.set());
      desired_set = intersect;
      break;
    }
  }
  const int order {3};
  const double h_min {0.0};
  const double h_max {20.0};

  q.push_back(
      drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(
          desired_set));

  auto terminal_cHull = ConvexHull(q);

  if (start) {
    auto start_point = &gcs_traj_opt_->AddRegions(
        drake::geometry::optimization::MakeConvexSets(*q[0]), 0, h_min, h_max,
        "start_point");
    start_region_ = &gcs_traj_opt_->AddRegions(
        drake::geometry::optimization::MakeConvexSets(terminal_cHull), order,
        h_min, h_max, "start_region");

    // Create a vector of pairs
    std::vector<std::pair<int, int>> edges_between_regions = {
        std::make_pair(0, 0)};

    gcs_traj_opt_->AddEdges(*start_point, *start_region_, nullptr,
                            &edges_between_regions);
    gcs_traj_opt_->AddEdges(*start_region_, *main_regions_);

  } else {
    auto end_point = &gcs_traj_opt_->AddRegions(
        drake::geometry::optimization::MakeConvexSets(*q[0]), 0, h_min, h_max,
        "end_point");
    auto end_region = &gcs_traj_opt_->AddRegions(
        drake::geometry::optimization::MakeConvexSets(terminal_cHull), order,
        h_min, h_max, "end_region");

    gcs_traj_opt_->AddEdges(*main_regions_, *end_region);

    gcs_traj_opt_->AddEdges(*start_region_, *end_region);

    // Create a vector of pairs
    std::vector<std::pair<int, int>> edges_between_regions = {
        std::make_pair(0, 0)};

    // Call the AddEdges function
    gcs_traj_opt_->AddEdges(*end_region, *end_point, nullptr,
                            &edges_between_regions);
  }
}

// Method to add terminal node
void GraphOfConvexHulls::AddTerminalNode(
    drake::geometry::optimization::ConvexSets q,
    const std::tuple<int, int, int> connect_sets, const bool start) {
  auto& intersections = iris_regions_adapter_.intersections_vec().value();

  auto desired_intersection = std::make_pair(
      std::min(std::get<0>(connect_sets), std::get<1>(connect_sets)),
      std::max(std::get<0>(connect_sets), std::get<1>(connect_sets)));

  int start_ind =
      std::min(std::get<0>(connect_sets), std::get<2>(connect_sets));
  int end_ind = std::max(std::get<0>(connect_sets), std::get<2>(connect_sets));
  int mid_ind = std::get<1>(connect_sets);
  int desired_cHull_ind =
      index_mapping_.at(std::make_tuple(start_ind, mid_ind, end_ind));

  drake::geometry::optimization::HPolyhedron desired_set;

  for (auto reg : intersections) {
    if (std::make_pair(reg.index_one(), reg.index_two())
        == desired_intersection) {
      const auto& set_1 =
          iris_regions_adapter_.regions_vec().at(reg.index_one());
      const auto& set_2 =
          iris_regions_adapter_.regions_vec().at(reg.index_two());
      const auto intersect = set_1.set().Intersection(set_2.set());
      desired_set = intersect;
      break;
    }
  }
  const int order {3};
  const double h_min {0.0};
  const double h_max {20.0};

  q.push_back(
      drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(
          desired_set));

  auto terminal_cHull = ConvexHull(q);

  if (start) {
    auto start_point = &gcs_traj_opt_->AddRegions(
        drake::geometry::optimization::MakeConvexSets(*q[0]), 0, h_min, h_max,
        "start_point");
    start_region_ = &gcs_traj_opt_->AddRegions(
        drake::geometry::optimization::MakeConvexSets(terminal_cHull), order,
        h_min, h_max, "start_region");

    // Create a vector of pairs
    std::vector<std::pair<int, int>> edges_between_regions = {
        std::make_pair(0, 0)};

    std::vector<std::pair<int, int>> edges_to_main = {
        std::make_pair(0, desired_cHull_ind)};

    gcs_traj_opt_->AddEdges(*start_point, *start_region_, nullptr,
                            &edges_between_regions);
    gcs_traj_opt_->AddEdges(*start_region_, *main_regions_, nullptr,
                            &edges_to_main);

  } else {
    auto end_point = &gcs_traj_opt_->AddRegions(
        drake::geometry::optimization::MakeConvexSets(*q[0]), 0, h_min, h_max,
        "end_point");
    auto end_region = &gcs_traj_opt_->AddRegions(
        drake::geometry::optimization::MakeConvexSets(terminal_cHull), order,
        h_min, h_max, "end_region");

    gcs_traj_opt_->AddEdges(*main_regions_, *end_region);

    gcs_traj_opt_->AddEdges(*start_region_, *end_region);

    // Create a vector of pairs
    std::vector<std::pair<int, int>> edges_between_regions = {
        std::make_pair(0, 0)};

    // Call the AddEdges function
    gcs_traj_opt_->AddEdges(*end_region, *end_point, nullptr,
                            &edges_between_regions);
  }
}

// Method to remove terminal node
void GraphOfConvexHulls::RemTerminalNode() {
  auto subgraphs = gcs_traj_opt_->GetSubgraphs();

  for (Subgraph* subgraph : subgraphs) {
    if (subgraph->name() != "main_regions") {
      gcs_traj_opt_->RemoveSubgraph(*subgraph);
    }
  }
}

}  // namespace planning
}  // namespace motion
