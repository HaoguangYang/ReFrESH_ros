#ifndef BT_REFRESH_ROS_ACTION_NODE_HPP
#define BT_REFRESH_ROS_ACTION_NODE_HPP

#include "bt_action_node.hpp"
#include "bt_refresh_control_node.hpp"
#include "bt_service_node.hpp"
#include "refresh_ros_msgs/action/self_adaptive_action.hpp"
#include "refresh_ros_msgs/srv/self_adaptive_action_estimate.hpp"

using refresh_ros_msgs::action::SelfAdaptiveAction;
using refresh_ros_msgs::srv::SelfAdaptiveActionEstimate;

using BT::NodeStatus;

namespace BT {

template <class ActionT>
class ActionEvaluatorNode : public StatefulActionNode {
 protected:
  ActionEvaluatorNode(const std::string& name, const NodeConfiguration& conf)
      : StatefulActionNode(name, conf) {}

 public:
  using ActionType = ActionT;
  using Feedback = typename ActionT::Feedback;

  ActionEvaluatorNode() = delete;

  virtual ~ActionEvaluatorNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterReFRESH_EV<DeriveClass>()
  static PortsList providedPorts() {
    return {InputPort<Feedback>("feedback"), OutputPort<float>("performance_cost"),
            OutputPort<float>("resource_cost")};
  }

  virtual NodeStatus spinOnce() = 0;

  inline NodeStatus spinOnceImpl() {
    Result fbRes;
    if (!(fbRes = getInput<Feedback>("feedback", fb_)))
      throw(
          RuntimeError("Action Evaluator Node missing required input [feedback]: ", fbRes.error()));
    NodeStatus status = spinOnce();
    setOutput("performance_cost", pCost_);
    setOutput("resource_cost", rCost_);
    setStatus(status);
    return status;
  }

  inline NodeStatus onStart() override {
    setStatus(NodeStatus::RUNNING);
    return spinOnceImpl();
  }

  /// method invoked by an action in the RUNNING state.
  inline NodeStatus onRunning() override { return spinOnceImpl(); }

  inline void onHalted() override {
    // TODO: what to do here?
    return;
  }

 protected:
  Feedback fb_;
  float pCost_, rCost_;
};

/// Method to register the evaluator into a factory.
template <class DerivedT>
static void RegisterActionEvaluator(BehaviorTreeFactory& factory,
                                    const std::string& registration_ID) {
  NodeBuilder builder = [](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(name, config);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = ActionEvaluatorNode<typename DerivedT::ActionType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());

  factory.registerBuilder(manifest, builder);
}

};  // namespace BT

namespace ReFRESH {
class ReFRESH_ROS_EX_node : public BT::RosActionNode<SelfAdaptiveAction> {
 public:
  ReFRESH_ROS_EX_node(const std::string& instance_name, const BT::NodeConfiguration& conf,
                      const BT::RosActionNodeParams& params,
                      typename std::shared_ptr<ActionClient> external_action_client = {})
      : RosActionNode<SelfAdaptiveAction>(instance_name, conf, params, external_action_client) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("action_request"), BT::InputPort<std::string>("arguments")};
  }

  bool sendGoal(Goal& goal) override;
};

class ReFRESH_ROS_EV_node : public BT::ActionEvaluatorNode<SelfAdaptiveAction> {
 public:
  ReFRESH_ROS_EV_node(const std::string& name, const BT::NodeConfiguration& conf)
      : BT::ActionEvaluatorNode<SelfAdaptiveAction>(name, conf) {}

  virtual NodeStatus spinOnce() override;
};

class ReFrESH_ROS_ES_node : public BT::RosServiceNode<SelfAdaptiveActionEstimate> {
 public:
  ReFrESH_ROS_ES_node(const std::string& instance_name, const BT::NodeConfiguration& conf,
                      const BT::RosServiceNodeParams& params,
                      typename std::shared_ptr<ServiceClient> external_service_client = {})
      : RosServiceNode<SelfAdaptiveActionEstimate>(instance_name, conf, params,
                                                   external_service_client) {}

  /// These ports will be added automatically if this Node is
  /// registered using RegisterReFRESH_EV<DeriveClass>()
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("action_request"), BT::InputPort<std::string>("arguments"),
            BT::OutputPort<float>("performance_cost"), BT::OutputPort<float>("resource_cost")};
  }

  virtual bool sendRequest(Request& request) override;

  virtual NodeStatus onResponse(const Response& rep) override;
};

}  // namespace ReFRESH

#endif
