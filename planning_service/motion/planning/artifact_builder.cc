#include "planning_service/motion/planning/artifact_builder.h"

#include "planning_service/common/string_utils.h"

namespace fs = std::filesystem;
namespace motion {
namespace planning {

ArtifactBuilder::ArtifactBuilder(
    std::unique_ptr<motion::planning::ompl::ThunderPlanner> thunder_planner,
    const iris::IrisBuilderOptions& iris_builder_options,
    const std::string& iris_regions_adapter_file, const fs::path& context_dir,
    const fs::path& problems_dir)
    : thunder_planner_ {std::move(thunder_planner)},
      iris_builder_ {std::make_unique<motion::iris::IrisBuilder>(
          thunder_planner_->planning_context()->robot_constraints(),
          iris_builder_options, iris_regions_adapter_file,
          thunder_planner_->vertices_confs(),
          thunder_planner_->SparseAdjacencyMatrixOfRoadmap())},
      context_dir_ {context_dir},
      problems_dir_ {problems_dir},
      config_problems_dir_ {problems_dir_ / "config_problems"} {
  if (!fs::is_directory(context_dir_)) {
    throw std::runtime_error(fmt::format(
        "ArtifactBuilder: Context directory {} does not exist!", context_dir_));
  }
  if (!fs::is_directory(config_problems_dir_)) {
    fs::create_directories(config_problems_dir_);
  }
}

void ArtifactBuilder::PopulateVerticesToRegionsCorrespondence() {
  const auto& regions {iris_builder_->adapter().regions_vec()};
  for (int vertex = 0; vertex < GetNumRoadmapVertices(); ++vertex) {
    // get vertex conf
    const auto vertex_conf {thunder_planner_->GetVertexConf(vertex)};
    for (int i = 0; i < drake::ssize(regions); ++i) {
      const auto& region {regions.at(i)};
      if (iris_builder_->inspector()
              .EvalConfigAgainstIrisRegion(vertex_conf, region, false, false)
              .inside) {
        correspondence_.roadmap_vertex_to_iris_regions_set_[vertex].insert(i);
        correspondence_.iris_region_to_roadmap_vertices_[i].insert(vertex);
      }
    }
  }
  // log the roadmap_vertex_to_iris_regions_set in a human-readable way
  for (const auto& [vertex, regions] : roadmap_vertex_to_iris_regions_set()) {
    std::vector<std::string> regions_strs;
    std::transform(regions.cbegin(), regions.cend(),
                   std::back_inserter(regions_strs), [](int region) {
                     return std::to_string(region);
                   });
    logging::log()->debug(
        "ArtifactBuilder:PopulateVerticesToRegionsCorrespondence: Vertex {} is "
        "covered by regions {}",
        vertex, common::utils::join_strings(regions_strs, ", "));
  }
}

bool ArtifactBuilder::IsProblemValid(const Eigen::VectorXd& q_start,
                                     const Eigen::VectorXd& q_goal) const {
  const auto& validity_checker =
      thunder_planner_->planning_context()->validity_checker();
  return validity_checker->IsPlanningProblemValid(q_start, q_goal);
}

void ArtifactBuilder::LoadValidateAndSaveFiles(
    const std::string& load_from_file, const std::string& save_to_file,
    const bool migrate_on_load) {
  // create new thunder instance
  ::ompl::tools::Thunder thunder_planner(thunder_planner_->space_information());
  thunder_planner.setup();
  thunder_planner.getExperienceDB()->getSPARSdb()->setMigrateRoadmapOnLoad(
      migrate_on_load);
  thunder_planner.getExperienceDB()->getSPARSdb()->setup();
  logging::log()->info("Migrate on load is {}",
                       thunder_planner.getExperienceDB()
                           ->getSPARSdb()
                           ->getMigrateRoadmapOnLoad());
  logging::log()->info("Loading roadmap from {}", load_from_file);
  thunder_planner.getExperienceDB()->load(load_from_file);
  thunder_planner.setFilePath(save_to_file);
  logging::log()->info(
      "Loaded database has {} vertices and {} edges",
      thunder_planner.getExperienceDB()->getSPARSdb()->getNumVertices(),
      thunder_planner.getExperienceDB()->getSPARSdb()->getNumEdges());
  logging::log()->info(
      "Existing roadmap has {} vertices and {} edges",
      thunder_planner_->experience_database()->getSPARSdb()->getNumVertices(),
      thunder_planner_->experience_database()->getSPARSdb()->getNumEdges());
  thunder_planner.save();
  PopulateVerticesToRegionsCorrespondence();
}

void ArtifactBuilder::BuildRegionsFromRoadmap(
    const iris::IrisBuilder::IrisMethod& method) {
  // get the number of vertices
  const auto num_vertices {thunder_planner().planner_data().numVertices()};
  for (unsigned int from_vertex = 0; from_vertex < num_vertices;
       ++from_vertex) {
    std::vector<unsigned int> edge_list;
    thunder_planner().planner_data().getEdges(from_vertex, edge_list);
    for (unsigned int to_vertex : edge_list) {
      motion::iris::conf_edge_t edge {
          thunder_planner_->GetVertexConf(from_vertex),
          thunder_planner_->GetVertexConf(to_vertex)};
      bool edge_covered {false};
      for (int i = 0; i < drake::ssize(iris_builder_->adapter().regions_vec());
           ++i) {
        const auto& region {iris_builder_->adapter().regions_vec().at(i)};
        // check if from_conf and to_conf are inside the region
        // check if the key from_vertex is in the map
        // roadmap_vertex_to_iris_regions_set, and if the region index is
        // already in the vector
        const auto from_vertex_inside_map {
            roadmap_vertex_to_iris_regions_set().contains(from_vertex)
            && roadmap_vertex_to_iris_regions_set()[from_vertex].count(i) > 0};
        const auto to_vertex_inside_map {
            roadmap_vertex_to_iris_regions_set().contains(to_vertex)
            && roadmap_vertex_to_iris_regions_set()[to_vertex].count(i) > 0};

        const bool from_vertex_inside {
            from_vertex_inside_map
            || iris_builder_->inspector()
                   .EvalConfigAgainstIrisRegion(edge.first, region, false,
                                                false)
                   .inside};
        const bool to_vertex_inside {
            to_vertex_inside_map
            || iris_builder_->inspector()
                   .EvalConfigAgainstIrisRegion(edge.second, region, false,
                                                false)
                   .inside};
        if (from_vertex_inside) {
          roadmap_vertex_to_iris_regions_set()[from_vertex].insert(i);
          iris_region_to_roadmap_vertices()[i].insert(from_vertex);
        }
        if (to_vertex_inside) {
          roadmap_vertex_to_iris_regions_set()[to_vertex].insert(i);
          iris_region_to_roadmap_vertices()[i].insert(to_vertex);
        }
        if (from_vertex_inside && to_vertex_inside) {
          edge_covered = true;
          // we don't break here to make sure we populate
          // roadmap_vertex_to_iris_regions_set
        }
      }
      if (edge_covered) {
        logging::log()->debug(
            "ArtifactBuilder:BuildRegionsFromRoadmap: Edge ({},{}) is covered "
            "by existing regions ",
            from_vertex, to_vertex);
        continue;
      }
      // build region from edge
      try {
        auto region_name {fmt::format("edge_{}_{}", from_vertex, to_vertex)};
        iris_builder_->BuildFromEdges({edge}, method, region_name);
      } catch (const std::exception& e) {
        logging::log()->error(
            "ArtifactBuilder:BuildRegionsFromRoadmap: Failed to build region "
            "from edge ({},{}) due to exception: {}",
            from_vertex, to_vertex, e.what());
      }
      PopulateVerticesToRegionsCorrespondence();
    }
  }
}

void ArtifactBuilder::BuildRegionsFromSystemConfs(
    const std::vector<Eigen::VectorXd>& conf_vec,
    const iris::IrisBuilder::IrisMethod& method) {
  // Track the coverage of the vertices.
  iris_builder_->SetPointsForIrisCoverageEvaluation(conf_vec);
  iris_builder_->BuildFromConfigs(conf_vec, method);
}

void ArtifactBuilder::BuildRegionsFromEdges(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>&
        conf_edge_vec,
    const iris::IrisBuilder::IrisMethod& method) {
  iris_builder_->BuildFromEdges(conf_edge_vec, method);
}

std::optional<std::vector<Eigen::VectorXd>>
ArtifactBuilder::GetSampleBasedSolution(const Eigen::VectorXd& q_start,
                                        const Eigen::VectorXd& q_goal,
                                        const bool try_recall,
                                        const bool save_solution) {
  SavePlanningProblem(q_start, q_goal);
  std::optional<std::vector<Eigen::VectorXd>> solution;
  if (try_recall) {
    solution = thunder_planner_->SolveRecallPlan(q_start, q_goal);
  }
  if (!solution.has_value()) {
    solution =
        thunder_planner_->SolveParallelPlan(q_start, q_goal, save_solution);
    if (solution.has_value()) {
      PopulateVerticesToRegionsCorrespondence();
    }
  }
  return solution;
}

std::pair<int, int> ArtifactBuilder::GetNumCoveredVerticesAndEdges() {
  PopulateVerticesToRegionsCorrespondence();

  const unsigned int num_vertices {
      static_cast<unsigned int>(GetNumRoadmapVertices())};
  int num_covered_vertices {0};
  int num_covered_edges {0};

  for (unsigned int from_vertex = 0; from_vertex < num_vertices;
       from_vertex++) {
    if (correspondence_.roadmap_vertex_to_iris_regions_set_.contains(
            from_vertex)) {
      ++num_covered_vertices;
    } else {
      continue;
    }
    const auto& connected_vertices {
        thunder_planner_->GetConnectedVertices({from_vertex})};
    for (unsigned int to_vertex : connected_vertices) {
      if (to_vertex < from_vertex
          || !correspondence_.roadmap_vertex_to_iris_regions_set_.contains(
              to_vertex)) {
        continue;
      }
      // check if there is a region present in both
      // roadmap_vertex_to_iris_regions_set().at(i) and
      // roadmap_vertex_to_iris_regions_set().at(j)
      const auto& from_vertex_regions {
          correspondence_.roadmap_vertex_to_iris_regions_set_.at(from_vertex)};
      const auto& to_vertex_regions {
          correspondence_.roadmap_vertex_to_iris_regions_set_.at(to_vertex)};
      for (const auto& region : from_vertex_regions) {
        if (to_vertex_regions.contains(region)) {
          ++num_covered_edges;
          break;
        }
      }
    }
  }
  return {num_covered_vertices, num_covered_edges};
}

const PlanningArtifactStatus ArtifactBuilder::GetPlanningArtifactStatus(
    const int num_samples) {
  PlanningArtifactStatus status;
  // PRM metrics
  const auto num_roadmap_vertices {
      thunder_planner_->experience_database()->getSPARSdb()->getNumVertices()};
  const auto num_connected_components {thunder_planner_->experience_database()
                                           ->getSPARSdb()
                                           ->getNumConnectedComponents()};
  const auto& num_roadmap_edges {thunder_planner_->GetRoadmapEdges().size()};
  status.num_roadmap_vertices = num_roadmap_vertices;
  status.num_roadmap_edges = num_roadmap_edges;
  status.num_connected_components = num_connected_components;

  const auto [num_covered_vertices,
              num_covered_edges] {GetNumCoveredVerticesAndEdges()};

  status.num_roadmap_covered_edges = num_covered_edges;
  status.num_roadmap_covered_vertices = num_covered_vertices;

  const auto& iris_regions {iris_builder_->adapter().regions_vec()};
  // IRIS metrics
  status.num_iris_regions = iris_regions.size();
  if (num_samples > 0) {
    drake::RandomGenerator generator {0};
    SampleOptions options;
    options.max_num_samples = std::max(10000, 2 * num_samples);
    const auto q_vec_samples {
        thunder_planner_->planning_context()
            ->robot_constraints()
            .GenerateSamples(&generator, num_samples, options)};
    iris_builder_->SetPointsForIrisCoverageEvaluation(q_vec_samples);
    const auto coverage {iris_builder_->EvaluateCoverage()};

    status.iris_volume_coverage = coverage.volume_covered;
    status.iris_visibility_coverage = coverage.visibility_covered;
    status.iris_uncovered = coverage.uncovered;
  }
  return status;
}

void ArtifactBuilder::RepairRegions(
    std::vector<Eigen::VectorXd> repair_configurations, int num_samples) {
  // get all regions in a vec
  const auto& regions {iris_builder_->adapter().regions_vec()};
  // iterate and repair
  std::vector<std::tuple<drake::geometry::optimization::HPolyhedron,
                         std::string, size_t>>
      updated_regions;
  int num_repaired_regions {0};
  logging::log()->info("ArtifactBuilder:RepairRegions: Repairing {} regions",
                       regions.size());
  omp_set_num_threads(thunder_planner_->planning_context()
                          ->validity_checker()
                          ->robot_constraints()
                          .num_threads());
  // robot constraints hash
  const auto constraints_hash {thunder_planner_->planning_context()
                                   ->robot_constraints()
                                   .constraints_hash()};
#pragma omp parallel for shared(updated_regions, num_repaired_regions)
  for (const auto& region : regions) {
    try {
      const auto thread_num {omp_get_thread_num()};
      const auto& [repaired_region, repair_required] =
          iris_builder_->RepairRegionViaSampling(
              region.set(), repair_configurations, num_samples, thread_num);
#pragma omp critical
      {
        updated_regions.emplace_back(repaired_region, region.name(),
                                     constraints_hash);
      }
      if (repair_required) {
        logging::log()->info(
            "ArtifactBuilder:RepairRegions: Repaired region: {}",
            region.name());
        ++num_repaired_regions;
      } else {
        logging::log()->debug(
            "ArtifactBuilder:RepairRegions: Region {} is valid", region.name());
      }

    } catch (const std::exception& e) {
      logging::log()->error(
          "ArtifactBuilder:RepairRegions: Failed to repair region {} due to "
          "exception: {}",
          region.name(), e.what());
      continue;
    }
  }
  // Add all regions after computation, then save to disk
  motion::iris::IrisRegionsAdapter iris_regions_adapter_repaired;
  for (const auto& [region, name, region_constraints_hash] : updated_regions) {
    iris_regions_adapter_repaired.AddRegion(region, name,
                                            region_constraints_hash);
  }
  iris_builder_->SetIrisRegionsAdapter(iris_regions_adapter_repaired);
  drake::yaml::SaveYamlFile(iris_builder_->adapter_file(),
                            iris_regions_adapter_repaired);
  logging::log()->info(
      "ArtifactBuilder:RepairRegions: Repaired {} regions out of {}",
      num_repaired_regions, regions.size());
  PopulateVerticesToRegionsCorrespondence();
}

void ArtifactBuilder::SavePlanningProblem(const Eigen::VectorXd& q_start,
                                          const Eigen::VectorXd& q_goal) {
  ConfigSpacePlanningProblem problem {q_start, q_goal};
  const auto problem_filename {fmt::format(
      "problem-{}.yaml", std::hash<ConfigSpacePlanningProblem>()(problem))};

  drake::yaml::SaveYamlFile(config_problems_dir_ / problem_filename, problem);
}

std::vector<ConfigSpacePlanningProblem> ArtifactBuilder::LoadPlanningProblems(
    std::optional<int> num_problems) {
  if (!num_problems.has_value()) {
    num_problems = std::numeric_limits<int>::max();
  } else if (num_problems.value() <= 0) {
    return {};
  }
  std::vector<ConfigSpacePlanningProblem> problems;
  for (const auto& filename : fs::directory_iterator(config_problems_dir_)) {
    ConfigSpacePlanningProblem problem;
    // if the entry is a .yaml file, load it
    if (filename.path().extension() == ".yaml") {
      problem = drake::yaml::LoadYamlFile<ConfigSpacePlanningProblem>(
          filename.path());
      problems.push_back(problem);
      if (static_cast<int>(problems.size()) >= num_problems.value()) {
        break;
      }
    }
  }
  return problems;
}

void ArtifactBuilder::UpdateRoadmapFromSavedProblems(
    std::optional<int> num_problems) {
  const auto problems {LoadPlanningProblems(num_problems)};
  if (problems.empty()) {
    logging::log()->info(
        "ArtifactBuilder:UpdateRoadmapFromSavedProblems: No problems to "
        "solve");
    return;
  }
  for (const auto& problem : problems) {
    // Check if the planning problem is valid
    if (!IsProblemValid(problem.q_start, problem.q_goal)) {
      logging::log()->warn(
          "ArtifactBuilder:UpdateRoadmapFromSavedProblems: Skipping "
          "invalid planning problem with start: [{}] and goal: [{}]",
          problem.q_start.transpose(), problem.q_goal.transpose());
      continue;
    }
    const bool try_recall {false};
    const bool save_solution {true};
    GetSampleBasedSolution(problem.q_start, problem.q_goal, try_recall,
                           save_solution);
  }
}

// Add this new helper function to ArtifactBuilder
bool ArtifactBuilder::SolveProblemAndBuildRegions(
    const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal,
    const iris::IrisBuilder::IrisMethod& method, const bool save_to_roadmap) {
  // Check if the planning problem is valid
  if (!IsProblemValid(q_start, q_goal)) {
    logging::log()->warn(
        "ArtifactBuilder:BuildRegionsFromSavedProblemsPath: Skipping "
        "invalid planning problem with start: [{}] and goal: [{}]",
        q_start.transpose(), q_goal.transpose());
    return false;
  }

  // Solve the planning problem to get a path
  const auto solution_path {GetSampleBasedSolution(
      q_start, q_goal, !save_to_roadmap, save_to_roadmap)};

  if (!solution_path.has_value()) {
    logging::log()->error(
        "ArtifactBuilder:BuildRegionsFromSavedProblemsPath: Failed to "
        "solve planning problem with start: [{}] and goal: [{}]",
        q_start.transpose(), q_goal.transpose());
    return false;
  }

  // Build IRIS regions from the solution path
  try {
    iris_builder_->BuildFromPath(solution_path.value(), method);
    logging::log()->info(
        "ArtifactBuilder:BuildRegionsFromSavedProblemsPath: Successfully "
        "built regions from problem with {} configurations in path",
        solution_path.value().size());
    return true;
  } catch (const std::exception& e) {
    logging::log()->error(
        "ArtifactBuilder:BuildRegionsFromSavedProblemsPath: Failed to "
        "build regions from path due to exception: {}",
        e.what());
    return false;
  }
}

void ArtifactBuilder::BuildRegionsFromSavedProblemsPath(
    int num_problems, const iris::IrisBuilder::IrisMethod& method) {
  const auto problems {LoadPlanningProblems(num_problems)};
  if (problems.empty()) {
    logging::log()->info(
        "ArtifactBuilder:BuildRegionsFromSavedProblemsPath: No problems to "
        "solve");
    return;
  }

  for (const auto& problem : problems) {
    SolveProblemAndBuildRegions(problem.q_start, problem.q_goal, method);
  }

  // Update correspondences after building new regions
  PopulateVerticesToRegionsCorrespondence();
}

/** Set the IRIS regions adapter */
void ArtifactBuilder::SetIrisRegionsAdapter(
    const iris::IrisRegionsAdapter& iris_regions_adapter) {
  iris_builder_->SetIrisRegionsAdapter(iris_regions_adapter);
  PopulateVerticesToRegionsCorrespondence();
}

std::string ArtifactBuilder::GetGraphvizString() {
  PopulateVerticesToRegionsCorrespondence();
  const auto adjaceny_matrix =
      thunder_planner_->SparseAdjacencyMatrixOfRoadmap();
  // Create a digraph. Color vertices and edges that are covered by IRIS
  // regions.
  std::string color_uncovered_vertex = "black";
  std::string color_uncovered_edge = "gray50";
  // The following colors will be randomly chosen based on the first region
  // index
  std::string color_covered_vertex, color_covered_edge;
  // First: make the vertices
  std::string result = "digraph G {\n";
  result += R"(
      node [
  shape=circle,
  width=0.01,
  height=0.01,
  fixedsize=true,
  label="",
  style=filled,
];
)";
  result += R"(
edge [
  penwidth=0.005,
  arrowsize=0.001,
];
)";
  std::vector<std::string> region_colors = {
      "red",    "green",  "blue", "yellow", "cyan", "magenta",
      "orange", "purple", "pink", "brown",  "gray"};
  int num_covered_vertices = 0;
  int num_uncovered_vertices = 0;
  int num_covered_edges = 0;
  int num_uncovered_edges = 0;
  int num_correspondance_not_prm = 0;
  int n = std::ssize(thunder_planner_->vertices_confs());
  for (int i = 0; i < n; ++i) {
    if (correspondence_.roadmap_vertex_to_iris_regions_set_.count(i) > 0
        && correspondence_.roadmap_vertex_to_iris_regions_set_.at(i).size()
               > 0) {
      color_covered_vertex = region_colors
          [*(correspondence_.roadmap_vertex_to_iris_regions_set_.at(i).begin())
           % region_colors.size()];
      ++num_covered_vertices;
      result += fmt::format("  v{} [color={}];\n", i, color_covered_vertex);
    } else {
      ++num_uncovered_vertices;
      result += fmt::format("  v{} [color={}];\n", i, color_uncovered_vertex);
    }
    // Check edges coming out of vertex i
    for (int j = i + 1; j < n; ++j) {
      bool correspondance_edge = false;
      if (correspondence_.roadmap_vertex_to_iris_regions_set_.count(i) > 0
          && correspondence_.roadmap_vertex_to_iris_regions_set_.count(j) > 0) {
        // If correspondance have a common region, color it differently
        std::set<int> intersection;
        std::set_intersection(
            correspondence_.roadmap_vertex_to_iris_regions_set_.at(i).begin(),
            correspondence_.roadmap_vertex_to_iris_regions_set_.at(i).end(),
            correspondence_.roadmap_vertex_to_iris_regions_set_.at(j).begin(),
            correspondence_.roadmap_vertex_to_iris_regions_set_.at(j).end(),
            std::inserter(intersection, intersection.begin()));
        correspondance_edge = !intersection.empty();
        color_covered_edge =
            region_colors[*(intersection.begin()) % region_colors.size()];
      }
      bool prm_edge =
          adjaceny_matrix.coeff(i, j) || adjaceny_matrix.coeff(j, i);
      if (correspondance_edge && !prm_edge) {
        ++num_correspondance_not_prm;
        result += fmt::format("  v{} -> v{} [color={}];\n", i, j, "red");
      } else if (prm_edge && correspondance_edge) {
        ++num_covered_edges;
        result +=
            fmt::format("  v{} -> v{} [color={}];\n", i, j, color_covered_edge);
      } else if (prm_edge) {
        ++num_uncovered_edges;
        result += fmt::format("  v{} -> v{} [color={}];\n", i, j,
                              color_uncovered_edge);
      }
    }
  }
  auto statistics = fmt::format(
      "Combined Artifacts statistics: \n{}/{} = {:.2f}% vertices covered "
      "\n{}/{} = {:.2f}% edges covered \n{} edges covered by regions but not "
      "in PRM\n",
      num_covered_vertices, num_covered_vertices + num_uncovered_vertices,
      100.0 * num_covered_vertices
          / (num_covered_vertices + num_uncovered_vertices),
      num_covered_edges, num_covered_edges + num_uncovered_edges,
      100.0 * num_covered_edges / (num_covered_edges + num_uncovered_edges),
      num_correspondance_not_prm);
  // Add the stats to the end of the result string as its label
  auto graph_str = fmt::format(
      "\nlabel=\"{}\";\nlabelloc=top;\nlabeljust=left;\nfontsize=10;\n",
      statistics);
  logging::log()->info(statistics);
  result += graph_str;
  result += "}\n";
  return result;
}

}  // namespace planning
}  // namespace motion
