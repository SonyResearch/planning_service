#include <drake/common/hash.h>
#include <drake/math/rigid_transform.h>
#include <eigen3/Eigen/Dense>

#include <map>
#include <set>

#include "planning_service/motion/iris/iris_builder.h"
#include "planning_service/motion/planning/thunder_planner.h"

namespace fs = std::filesystem;
namespace motion {
namespace planning {

/** Struct containing information about a given planner's planning artifacts. */
struct PlanningArtifactStatus {
  // PRM coverage metrics
  int num_roadmap_vertices;
  int num_roadmap_edges;
  int num_connected_components;
  int num_roadmap_covered_vertices;
  int num_roadmap_covered_edges;
  // IRIS coverage metrics
  int num_iris_regions;
  int iris_volume_coverage {-1};
  int iris_visibility_coverage {-1};
  int iris_uncovered {-1};
};

struct ConfigSpacePlanningProblem {
  Eigen::VectorXd q_start;
  Eigen::VectorXd q_goal;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(q_start));
    a->Visit(DRAKE_NVP(q_goal));
  }
};

class ArtifactBuilder {
 public:
  // Constructor that takes a thunder planner, and iris_builder_options,
  // iris_regions_adapter_file, then populates the clique_configs from the
  // roadmap vertices, and the adjacency matrix from the roadmap adjacency
  // matrix.
  ArtifactBuilder(
      std::unique_ptr<motion::planning::ompl::ThunderPlanner> thunder_planner,
      const iris::IrisBuilderOptions& iris_builder_options,
      const std::string& iris_regions_adapter_file, const fs::path& context_dir,
      const fs::path& problems_dir);

  /**
   * @brief Struct containing the correspondence between PRM vertices
   * and IRIS regions. `Correspondence` is defined as coverage of a given vertex
   * by a given region, i.e., the vertex is inside the region.
   *
   */
  struct VerticesToRegionsCorrespondence {
    std::map<int, std::set<int>> roadmap_vertex_to_iris_regions_set_;
    std::map<int, std::set<int>> iris_region_to_roadmap_vertices_;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(roadmap_vertex_to_iris_regions_set_));
      a->Visit(DRAKE_NVP(iris_region_to_roadmap_vertices_));
    }
  };

  /**
   * @brief Populate the roadmap vertices to IRIS regions correspondence by
   * checking each vertex and region for inclusion
   */
  void PopulateVerticesToRegionsCorrespondence();

  const std::set<int>& GetRegionsContainingVertex(int vertex) const {
    return roadmap_vertex_to_iris_regions_set().at(vertex);
  }

  const std::set<int>& GetVerticesContainedInRegion(int region) const {
    return iris_region_to_roadmap_vertices().at(region);
  }

  void AddConfsToRoadmap(const std::vector<Eigen::VectorXd>& confs) {
    thunder_planner_->AddConfsToRoadmap(confs);
    // repopulate the correspondence since a new vertex was added
    PopulateVerticesToRegionsCorrespondence();
  }

  /**
   * @brief Get a recalled plan from the roadmap. If a plan solution can't be
   * recalled, plan from scratch. Do not attempt to insert solution in the
   * roadmap, as this function is expected to be called during runtime.
   * @param start_conf The start configuration
   * @param goal_conf The goal configuration
   */
  std::optional<std::vector<Eigen::VectorXd>> GetSampleBasedSolution(
      const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal,
      const bool try_recall = true, const bool save_solution = false);

  /**
   * @brief Build regions from caller-passed robot configurations
   *
   * @param conf_vec the configurations provided by the caller
   */
  void BuildRegionsFromSystemConfs(const std::vector<Eigen::VectorXd>& conf_vec,
                                   const iris::IrisBuilder::IrisMethod& method =
                                       iris::IrisBuilder::IrisMethod::kIrisNp2);

  /** Build IRIS regions from a vector of configuration edges provided by
   * caller. */
  void BuildRegionsFromEdges(
      const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>&
          conf_edge_vec,
      const iris::IrisBuilder::IrisMethod& method =
          iris::IrisBuilder::IrisMethod::kIrisNp2);

  /**
   * @brief Build IRIS regions from an RRT-based probabilistic roadmap, by
   * breaking it down into edges (or largest cliques containing edges). The
   * built regions will cover all of the roadmap at algorithm completion.
   * If regions are already available, edges will be checked for containement in
   * those regions. Generation will only occur when an edge is not covered by
   * existing regions.
   * @param method The method to use for building the regions (Can be IRIS-NP,
   * IRIS-NP2, IRIS-ZO)
   */
  void BuildRegionsFromRoadmap(const iris::IrisBuilder::IrisMethod& method =
                                   iris::IrisBuilder::IrisMethod::kIrisNp2);

  /**
   * @brief Validate roadmap by migrating it on load and saving it again
   *
   */
  void LoadValidateAndSaveFiles(const std::string& load_from_file,
                                const std::string& save_to_file,
                                const bool migrate_on_load = true);

  /**
   * @brief Return the status of the planning artifacts.
   * @param num_samples Optional number of samples with which to perform IRIS
   * coverage evaluation (each sample can be volume covered, visibility covered,
   * or uncovered). If not provided, no IRIS evaluation is performed, and only
   * size results will be returned (i.e., number of regions, vertices, edges).
   *
   */
  const PlanningArtifactStatus GetPlanningArtifactStatus(
      const int num_samples = 0);

  /**
   * @brief Repair regions via sampling
   * @param repair_configurations Configurations to use for repairing regions.
   * @param num_samples Number of samples to take from each region for repair.
   */
  void RepairRegions(std::vector<Eigen::VectorXd> repair_configurations = {},
                     const int num_samples = 5000);

  /**
   * @brief Return the roadmap edges
   *
   * @return Roadmap edges
   */
  const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>
  GetRoadmapEdges() const {
    return thunder_planner_->GetRoadmapEdges();
  }

  int GetNumRegions() const {
    return iris_builder_->adapter().regions_vec().size();
  }

  int GetNumRoadmapVertices() const {
    return thunder_planner_->planner_data().numVertices();
  }

  void SavePlanningProblem(const Eigen::VectorXd& q_start,
                           const Eigen::VectorXd& q_goal);

  std::vector<ConfigSpacePlanningProblem> LoadPlanningProblems(
      std::optional<int> num_problems = std::nullopt);

  /**
   * @brief Check if a planning problem is valid
   *
   * Validates that both the start and goal configurations are valid according
   * to the robot constraints. Uses the ValidityChecker to perform the
   * validation.
   *
   * @param q_start Start configuration
   * @param q_goal Goal configuration
   * @return true if both start and goal configurations are valid, false
   * otherwise
   */
  bool IsProblemValid(const Eigen::VectorXd& q_start,
                      const Eigen::VectorXd& q_goal) const;

  void UpdateRoadmapFromSavedProblems(
      std::optional<int> num_problems = std::nullopt);

  /**
   * @brief Solve a planning problem, possibly save the solution to the roadmap,
   * and build IRIS regions that cover the solution path.
   * @param q_start The start configuration
   * @param q_goal The goal configuration
   * @param method The IRIS method to use for building regions
   * @param save_to_roadmap If true, the solution path will be saved to the
   * roadmap. If false, the solution will not be saved to the roadmap.
   * @return true if the problem was solved and regions were built successfully,
   * false otherwise.
   */
  bool SolveProblemAndBuildRegions(const Eigen::VectorXd& q_start,
                                   const Eigen::VectorXd& q_goal,
                                   const iris::IrisBuilder::IrisMethod& method,
                                   const bool save_to_roadmap = false);

  /**
   * @brief Build IRIS regions from saved planning problems by solving them
   * and padding the solution paths with regions
   *
   * Loads planning problems from disk, validates each problem, solves it to
   * get a solution path, then uses BuildFromPath to create IRIS regions that
   * cover the path. This is useful for building regions that cover commonly
   * used paths in the workspace.
   *
   * @param num_problems Number of problems to process. If the saved problems
   * contain fewer problems, all available problems will be processed.
   * @param method The IRIS method to use for building regions
   */
  void BuildRegionsFromSavedProblemsPath(
      int num_problems, const iris::IrisBuilder::IrisMethod& method =
                            iris::IrisBuilder::IrisMethod::kIrisNp2);

  /** Set the IRIS regions adapter */
  void SetIrisRegionsAdapter(
      const iris::IrisRegionsAdapter& iris_regions_adapter);

  /**
   * @brief Get the Num Covered Vertices And Edges object
   *
   * @return std::pair<int, int> representing the number of covered vertices and
   * edges by the IRIS regions
   */
  std::pair<int, int> GetNumCoveredVerticesAndEdges();

  // iris builder getter
  const motion::iris::IrisBuilder& iris_builder() const {
    return *iris_builder_;
  }

  // thunder planner getter
  const motion::planning::ompl::ThunderPlanner& thunder_planner() const {
    return *thunder_planner_;
  }

  // Get mutable thunder planner
  motion::planning::ompl::ThunderPlanner& mutable_thunder_planner() {
    return *thunder_planner_;
  }

  std::map<int, std::set<int>> roadmap_vertex_to_iris_regions_set() const {
    return correspondence_.roadmap_vertex_to_iris_regions_set_;
  }

  std::map<int, std::set<int>> iris_region_to_roadmap_vertices() const {
    return correspondence_.iris_region_to_roadmap_vertices_;
  }

  const fs::path& context_dir() const {
    return context_dir_;
  }

  const fs::path& config_problems_dir() const {
    return config_problems_dir_;
  }

  std::string GetGraphvizString();

 private:
  std::unique_ptr<motion::planning::ompl::ThunderPlanner> thunder_planner_;
  std::unique_ptr<iris::IrisBuilder> iris_builder_;
  VerticesToRegionsCorrespondence correspondence_;
  const fs::path context_dir_;
  // Directories containing planning problems
  const fs::path problems_dir_;
  const fs::path config_problems_dir_;
};

}  // namespace planning
}  // namespace motion

template <>
struct std::hash<Eigen::VectorXd> {
  std::size_t operator()(const Eigen::VectorXd& vec) const {
    std::size_t seed = 0;
    for (int i = 0; i < vec.size(); ++i) {
      seed ^=
          std::hash<double>()(vec[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

template <>
struct std::hash<motion::planning::ConfigSpacePlanningProblem> {
  std::size_t operator()(
      const motion::planning::ConfigSpacePlanningProblem& cspp) const {
    std::size_t seed = 0;

    seed ^= std::hash<Eigen::VectorXd>()(cspp.q_start) + 0x9e3779b9
            + (seed << 6) + (seed >> 2);
    seed ^= std::hash<Eigen::VectorXd>()(cspp.q_goal) + 0x9e3779b9 + (seed << 6)
            + (seed >> 2);
    return seed;
  }
};

template <>
struct fmt::formatter<motion::planning::PlanningArtifactStatus> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(motion::planning::PlanningArtifactStatus const& s,
              FormatContext& ctx) const {
    std::string format_msg {
        fmt::format("Roadmap: {} vertices, {} edges, covered_vertices: {}/{}, "
                    "covered_edges: {}/{}, "
                    "connected components: {}\n",
                    s.num_roadmap_vertices, s.num_roadmap_edges,
                    s.num_roadmap_covered_vertices, s.num_roadmap_vertices,
                    s.num_roadmap_covered_edges, s.num_roadmap_edges,
                    s.num_connected_components)};
    format_msg += fmt::format("IRIS regions: {}\n", s.num_iris_regions);
    int num_samples {s.iris_volume_coverage + s.iris_visibility_coverage
                     + s.iris_uncovered};
    if (s.iris_volume_coverage >= 0) {
      format_msg += fmt::format("\tvolume_coverage: {}/{}\n",
                                s.iris_volume_coverage, num_samples);
    }
    if (s.iris_visibility_coverage >= 0) {
      format_msg += fmt::format("\tvisibility_coverage: {}/{}\n",
                                s.iris_visibility_coverage, num_samples);
    }
    if (s.iris_uncovered >= 0) {
      format_msg +=
          fmt::format("\tuncovered: {}/{}", s.iris_uncovered, num_samples);
    }
    return fmt::format_to(ctx.out(), "{}", format_msg);
  }
};
