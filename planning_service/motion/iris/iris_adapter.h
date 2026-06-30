/*
 * Copyright © 2025 Sony Group Corporation. All rights reserved.
 */

/// @file iris_builder.h

#pragma once

#include <drake/common/random.h>
#include <drake/common/schema/stochastic.h>
#include <drake/common/type_safe_index.h>
#include <drake/geometry/optimization/convex_set.h>
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/hyperrectangle.h>
#include <drake/geometry/optimization/iris.h>
#include <drake/geometry/optimization/vpolytope.h>

#include <filesystem>

#include "planning_service/common/logging.h"

namespace motion {
namespace iris {

using convex_set_ptr_t =
    drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>;

/** An IrisRegionsAdapter is a collection of IrisRegions. It also store the
 * intersections. */
class IrisRegionsAdapter {
 public:
  /** Makes an empty IrisRegionsAdapter object */
  IrisRegionsAdapter() = default;

  /** An IrisRegion is a HPolyhedron in the configuration space of the robot
such that all of its points are satisfying the given robot constraints. This
class is a light wrapper around the HPolyhedron class from drake.
*/
  class IrisRegion {
   public:
    /** Makes an empty IrisRegion object */
    IrisRegion() = default;

    IrisRegion(drake::geometry::optimization::HPolyhedron set, int index,
               std::string name, size_t constraints_hash);

    /** Returns the HPolyhedron of the iris region. */
    const drake::geometry::optimization::HPolyhedron& set() const {
      return set_;
    }

    /** Returns the index of the iris region. */
    int index() const {
      return index_;
    }

    /** Returns the name of the iris region. */
    const std::string& name() const {
      return name_;
    }

    /** Returns the hash of the constraints used to build the iris region. */
    size_t constraints_hash() const {
      return constraints_hash_;
    }

    /** Returns the axis-aligned bounding box of the iris region. */
    const std::optional<drake::geometry::optimization::Hyperrectangle>& aabb()
        const {
      return aabb_;
    }

    /** Serializes the IrisRegion object */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(set_));
      a->Visit(DRAKE_NVP(index_));
      a->Visit(DRAKE_NVP(name_));
      a->Visit(DRAKE_NVP(constraints_hash_));
      a->Visit(DRAKE_NVP(aabb_));
    }

    friend class IrisBuilder;
    friend class IrisRegionsAdapter;

   private:
    drake::geometry::optimization::HPolyhedron set_ {
        drake::geometry::optimization::HPolyhedron()};
    int index_ {0};
    std::string name_ {"name"};
    size_t constraints_hash_ {0};
    std::optional<drake::geometry::optimization::Hyperrectangle> aabb_ {
        std::nullopt};
  };

  /** An IrisRegion is a HPolyhedron in the configuration space of the robot
  such that all of its points are satisfying the given robot constraints. This
  class is a light wrapper around the HPolyhedron class from drake.
  */
  class IrisRegionsIntersection {
   public:
    /** Makes an empty IrisRegion object */
    IrisRegionsIntersection() = default;

    /** Constructs an IrisRegionsIntersection object with the given parameters.
     * @param index_one the index of the first iris region
     * @param index_two the index of the second iris region
     * @param offset the offset to apply to the second iris region
     * @param intersection_samples the samples from the intersection of the two
     * regions */
    IrisRegionsIntersection(
        int index_one, int index_two, Eigen::VectorXd offset = {},
        std::vector<Eigen::VectorXd> intersection_samples = {})
        : index_one_(index_one),
          index_two_(index_two),
          offset_(offset),
          intersection_samples_(intersection_samples) {}

    /** Returns the index of the iris region. */
    int index_one() const {
      return index_one_;
    }

    /** Returns the index of the iris region. */
    int index_two() const {
      return index_two_;
    }

    /** Returns the offset of the iris region. */
    const Eigen::VectorXd& offset() const {
      return offset_;
    }

    /** Returns the intersection samples of the iris region. */
    const std::vector<Eigen::VectorXd>& intersection_samples() const {
      return intersection_samples_;
    }

    /** Serializes the IrisRegion object */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(index_one_));
      a->Visit(DRAKE_NVP(index_two_));
      a->Visit(DRAKE_NVP(offset_));
      a->Visit(DRAKE_NVP(intersection_samples_));
    }

    friend class IrisBuilder;
    friend class IrisRegionsAdapter;

   private:
    int index_one_ {0};
    int index_two_ {0};
    Eigen::VectorXd offset_ {Eigen::VectorXd::Zero(0)};
    std::vector<Eigen::VectorXd> intersection_samples_ {};
  };

  /** Adds an Iris Region. Returns a constant pointer to the added region. */
  const IrisRegion* AddRegion(
      const drake::geometry::optimization::HPolyhedron& set,
      const std::string& name, const size_t constraints_hash = 0,
      const std::vector<int>& continuous_revolute_joint_indices = {},
      int intersection_samples = 1, int mixing_steps = 0);

  /** Returns the iris regions */
  const std::vector<IrisRegion>& regions_vec() const {
    return regions_vec_;
  }

  const std::optional<std::vector<IrisRegionsIntersection>>& intersections_vec()
      const {
    return intersections_vec_;
  }

  /** Returns the convex sets as a vector of pointers */
  drake::geometry::optimization::ConvexSets GetConvexSets() const;

  /** Returns the string to be used in GraphViz */
  std::string CalcGraphVizString() const;

  /** @brief Calculates the intersection, and random intersection samples,
   * between two convex sets
   * @param a the first convex set
   * @param b the second convex set
   * @param offset the offset to apply to the second convex set
   * @param sample_count the number of samples to take from the intersection
   * @param mixing_steps the number of samples to skip in-between samples for
   * unpredictability
   * */
  std::pair<drake::geometry::optimization::HPolyhedron,
            std::vector<Eigen::VectorXd>>
  CalcIntersectionSamples(const drake::geometry::optimization::ConvexSet& a,
                          const drake::geometry::optimization::ConvexSet& b,
                          const Eigen::VectorXd& offset,
                          const int sample_count = 1,
                          const int mixing_steps = 0) const;

  /** Evaluates the intersection of an edge with a region.
  @param edge the edge to check that is parameterized by two configurations q₁,
  q₂.
  @param region the region to check.
  @return If the edge intersects the region, then the pair of two numbers λ₁, λ₂
  ∈ [0,1]. The set that is inside the region is given by {(1 - λ)q₁ + λq₂ | λ ∈
  [λ₁, λ₂]}. If the edge does not intersect the region, then nullopt is
  returned.
  */
  static std::optional<std::pair<double, double>>
  CalcEdgeIntersectionWithRegion(
      const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge,
      const drake::geometry::optimization::HPolyhedron& region);

  /** Serializes the IrisRegionsAdapter object */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(regions_vec_));
    a->Visit(DRAKE_NVP(intersections_vec_));

    if (!intersections_vec_.has_value() && !regions_vec_.empty()) {
      const auto regions_vec_copy = regions_vec_;
      *this = IrisRegionsAdapter();  // Reset the adapter to an empty state
      // Re-add the regions to the empty adapter.
      // Need to populate intersections_vec_ if it is not set.
      logging::log()->warn(
          "IrisRegionsAdapter::Serialize: intersections_vec_ is not set, "
          "populating it with no continuous revolute joints and 1 "
          "intersection samples.");
      // Need to add intersections to "this" object.
      for (int i = 0; i < std::ssize(regions_vec_copy); ++i) {
        const auto& region = regions_vec_copy.at(i);
        this->AddRegion(region.set(), region.name(), region.constraints_hash());
      }
      DRAKE_DEMAND(intersections_vec_.has_value());
      logging::log()->info(
          "IrisRegionsAdapter::Serialize: populated intersections_vec_ with "
          "empty intersections. Now it has {} regions and {} intersections.",
          regions_vec_.size(), intersections_vec_.value().size());
    }
  }

  friend class IrisBuilder;

 private:
  std::vector<IrisRegion> regions_vec_ {};
  std::optional<std::vector<IrisRegionsIntersection>> intersections_vec_ =
      std::nullopt;
};

}  // namespace iris
}  // namespace motion
