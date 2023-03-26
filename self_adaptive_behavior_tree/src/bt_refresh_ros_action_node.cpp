#include "self_adaptive_behavior_tree/bt_refresh_ros_action_node.hpp"

namespace ReFRESH_BT {
bool ROS_Action_EX_Node::sendGoal(Goal &goal) {
  if (!getInput<std::string>("action_request", goal.request)) {
    // abort the entire action. Result in a FAILURE
    return false;
  }
  if (!getInput<std::string>("arguments", goal.arguments)) {
    // abort the entire action. Result in a FAILURE
    return false;
  }
  return true;
}

NodeStatus ROS_Action_EV_Node::spinOnce() {
  std::vector<ReFRESH_Cost> allModuleCosts;
  allModuleCosts.reserve(fb_.evaluate.size());
  std::transform(fb_.evaluate.begin(), fb_.evaluate.end(), allModuleCosts.begin(),
                 [](const ModuleTelemetry &item) { return ReFRESH_Cost(0, item); });
  std::vector<float> allPerfCosts;
  allPerfCosts.reserve(fb_.evaluate.size());
  std::transform(allModuleCosts.begin(), allModuleCosts.end(), allPerfCosts.begin(),
                 [](const ReFRESH_Cost &in) { return in.performanceBottleneck(); });
  std::vector<float> allResCosts;
  allResCosts.reserve(fb_.evaluate.size());
  std::transform(allModuleCosts.begin(), allModuleCosts.end(), allResCosts.begin(),
                 [](const ReFRESH_Cost &in) { return in.resourceBottleneck(); });
  pCost_ = ReFRESH_Cost::max(allPerfCosts);
  rCost_ = ReFRESH_Cost::max(allResCosts);
  if (pCost_ > ReFRESH_Cost::BOUNDARY_ACCEPT) return NodeStatus::RUNNING;
  if (rCost_ > ReFRESH_Cost::BOUNDARY_ACCEPT) return NodeStatus::RUNNING;
  return NodeStatus::SUCCESS;
}

bool ROS_Action_ES_Node::sendRequest(Request &request) {
  if (!getInput<std::string>("action_request", request.request)) return false;
  if (!getInput<std::string>("arguments", request.arguments)) return false;
  return true;
}

NodeStatus ROS_Action_ES_Node::onResponse(const Response &rep) {
  std::vector<ReFRESH_Cost> allModuleCosts;
  allModuleCosts.reserve(rep.estimate.size());
  std::transform(rep.estimate.begin(), rep.estimate.end(), allModuleCosts.begin(),
                 [](const ModuleTelemetry &item) { return ReFRESH_Cost(0, item); });
  std::vector<float> allPerfCosts;
  allPerfCosts.reserve(rep.estimate.size());
  std::transform(allModuleCosts.begin(), allModuleCosts.end(), allPerfCosts.begin(),
                 [](const ReFRESH_Cost &in) { return in.performanceBottleneck(); });
  std::vector<float> allResCosts;
  allResCosts.reserve(rep.estimate.size());
  std::transform(allModuleCosts.begin(), allModuleCosts.end(), allResCosts.begin(),
                 [](const ReFRESH_Cost &in) { return in.resourceBottleneck(); });
  float allPerfBottleneck = ReFRESH_Cost::max(allPerfCosts);
  float allResBottleneck = ReFRESH_Cost::max(allResCosts);
  setOutput("performance_cost", allPerfBottleneck);
  setOutput("resource_cost", allResBottleneck);
  if (allPerfBottleneck >= 1.0) return NodeStatus::FAILURE;
  if (allResBottleneck >= 1.0) return NodeStatus::FAILURE;
  return NodeStatus::SUCCESS;
}
}  // namespace ReFRESH_BT
