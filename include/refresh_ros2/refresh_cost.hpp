#ifndef _REFRESH_COST_HPP_
#define _REFRESH_COST_HPP_

#include <float.h>

#include <vector>

#include "refresh_ros_msgs/msg/module_cost.hpp"
#include "refresh_ros_msgs/msg/module_telemetry.hpp"

using refresh_ros_msgs::msg::ModuleCost;
using refresh_ros_msgs::msg::ModuleTelemetry;

namespace ReFRESH {
class ReFRESH_Cost {
 public:
  static constexpr float REJECT = __FLT_MAX__;
  static constexpr float BOUNDARY_ACCEPT = 1.;
  static constexpr float BOUNDARY_REJECT = BOUNDARY_ACCEPT + __FLT_EPSILON__;
  static constexpr float NEXT_TO_BOUNDARY_ACCEPT = BOUNDARY_ACCEPT - __FLT_EPSILON__;

  template <typename T>
  static T max(const std::vector<T>& in) {
    return *std::max_element(in.begin(), in.end());
  }

  static float getCost(const ModuleCost& in) {
    return in.cost_normalized ? in.cost : in.cost / in.tolerance;
  };

  ReFRESH_Cost()
      : costUpdated_(false),
        childId_(0),
        performanceWeight_(0.5),
        resourceWeight_(0.5),
        performanceCost_{0.},
        resourceCost_{0.} {}

  ReFRESH_Cost(const size_t& index, const float& pCost, const float& rCost)
      : costUpdated_(false),
        childId_(0),
        performanceWeight_(0.5),
        resourceWeight_(0.5),
        performanceCost_{pCost},
        resourceCost_{rCost} {}

  ReFRESH_Cost(const size_t& index, const ModuleTelemetry& msg)
      : costUpdated_(false), childId_(index), performanceWeight_(0.5), resourceWeight_(0.5) {
    setPerformanceCost(msg.performance_cost);
    setResourceCost(msg.resource_cost);
  }

  void setPerformanceWeight(const float& pWeight) {
    costUpdated_ = false;
    performanceWeight_ = pWeight;
  }

  void setResourceWeight(const float& rWeight) {
    costUpdated_ = false;
    resourceWeight_ = rWeight;
  }

  void setPerformanceCost(const float& pCost) {
    costUpdated_ = false;
    performanceCost_ = {pCost};
    performanceBottleneck_ = pCost;
  }

  void setPerformanceCost(const std::vector<float>& pCost) {
    costUpdated_ = false;
    performanceCost_ = pCost;
    performanceBottleneck_ = max(pCost);
  }

  void setPerformanceCost(const std::vector<ModuleCost>& pCost) {
    costUpdated_ = false;
    performanceCost_.reserve(pCost.size());
    std::transform(pCost.begin(), pCost.end(), performanceCost_.begin(),
                   std::bind(&ReFRESH_Cost::getCost, std::placeholders::_1));
    performanceBottleneck_ = max(performanceCost_);
  }

  void setResourceCost(const float& rCost) {
    costUpdated_ = false;
    resourceCost_ = {rCost};
    resourceBottleneck_ = rCost;
  }

  void setResourceCost(const std::vector<float>& rCost) {
    costUpdated_ = false;
    resourceCost_ = rCost;
    resourceBottleneck_ = max(rCost);
  }

  void setResourceCost(const std::vector<ModuleCost>& rCost) {
    costUpdated_ = false;
    resourceCost_.reserve(rCost.size());
    std::transform(rCost.begin(), rCost.end(), resourceCost_.begin(),
                   std::bind(&ReFRESH_Cost::getCost, std::placeholders::_1));
    resourceBottleneck_ = max(resourceCost_);
  }

  void updateWeightedCost() const {
    if (costUpdated_) return;
    weightedCost_ =
        performanceWeight_ * performanceBottleneck_ + resourceWeight_ * resourceBottleneck_;
    costUpdated_ = true;
  }

  template <typename T>
  void updateCosts(const T& pCost, const T& rCost) {
    setPerformanceCost(pCost);
    setResourceCost(rCost);
    updateWeightedCost();
  }

  void setWeights(float& pWeight, float& rWeight) {
    setPerformanceWeight(pWeight);
    setResourceWeight(rWeight);
    updateWeightedCost();
  }

  void setChildId(const size_t& id) { childId_ = id; }

  size_t whichChild() const { return childId_; }

  float weightedCost() const { return weightedCost_; }

  float performanceBottleneck() const { return performanceBottleneck_; }

  float resourceBottleneck() const { return resourceBottleneck_; }

  bool resourceFeasible() const { return resourceBottleneck_ <= BOUNDARY_ACCEPT; }

  bool performanceFeasible() const { return performanceBottleneck_ <= BOUNDARY_ACCEPT; }

  bool feasible() const { return performanceFeasible() && resourceFeasible(); }

  /**
   * @brief Method used to compare the optimality between two module costs. If the first entry is
   * smaller and feasible, returns true.
   *
   * @param lhs first Self Adaptive Module Cost entry
   * @param rhs second Self Adaptive Module Cost entry
   * @return true The first entry is more optimal than the second entry.
   * @return false The first entry is NOT more optimal than the second entry.
   */
  bool operator< (const ReFRESH_Cost& rhs) const {
    updateWeightedCost();
    rhs.updateWeightedCost();
    return weightedCost_ < rhs.weightedCost() && feasible();
  };

 private:
  mutable bool costUpdated_ = false;
  mutable float weightedCost_, performanceBottleneck_, resourceBottleneck_;
  size_t childId_;
  float performanceWeight_, resourceWeight_;
  std::vector<float> performanceCost_{}, resourceCost_{};
};

}  // namespace ReFRESH

#endif  // _REFRESH_COST_HPP_
