#ifndef BT_REFRESH_CONTROL_NODE_HPP
#define BT_REFRESH_CONTROL_NODE_HPP

#include <float.h>

#include <vector>

#include "behaviortree_cpp_v3/control_node.h"
#include "bt_refresh_module_node.hpp"
#include "refresh_cost.hpp"

using BT::NodeStatus;

namespace ReFRESH {
class ReFRESH_Decider : public BT::ControlNode {
 public:
  ReFRESH_Decider(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ControlNode(name, config), indActive_(-1), bestPossible_(0) {
    nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0);
  }

  virtual ~ReFRESH_Decider() override = default;

  virtual void halt() override;

  // virtual bool isDepleted() { return false; }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<float>("performance_weight"), BT::InputPort<float>("resource_weight"),
            BT::InputPort<unsigned int>("retries", 3, "Number of retries for each child upon failure"),
            BT::InputPort<bool>("fallback_no_reconfig")};
  }

  NodeStatus turnOnBest();

 private:
  int indActive_;

  int bestPossible_;

  std::vector<unsigned int> nRetry_;

  ReFRESH_Cost activeModuleCost_;

  virtual NodeStatus tick() override;
};

class ReFRESH_Reactor : public BT::ControlNode {
 public:
  ReFRESH_Reactor(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ControlNode(name, config), indActive_(-1), bestPossible_(0) {
    nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0);
  }

  virtual ~ReFRESH_Reactor() override = default;

  virtual void halt() override;

  // virtual bool isDepleted() { return false; }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<float>("performance_weight"), BT::InputPort<float>("resource_weight"),
            BT::InputPort<unsigned int>("retries", 3, "Number of retries for each child upon failure"),
            BT::InputPort<bool>("fallback_no_reconfig")};
  }

  NodeStatus turnOnBestMitigation();

  NodeStatus turnOnNominal();

 private:
  int indActive_;

  int bestPossible_;

  std::vector<unsigned int> nRetry_;

  ReFRESH_Cost activeModuleCost_;

  virtual NodeStatus tick() override;
};

}  // namespace BT

#endif
