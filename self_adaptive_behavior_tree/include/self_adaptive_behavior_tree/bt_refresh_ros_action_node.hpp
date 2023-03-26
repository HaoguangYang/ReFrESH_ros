#ifndef BT_REFRESH_ROS_ACTION_NODE_HPP
#define BT_REFRESH_ROS_ACTION_NODE_HPP

#include "refresh_ros_msgs/action/self_adaptive_action.hpp"
#include "refresh_ros_msgs/srv/self_adaptive_action_estimate.hpp"
#include "self_adaptive_behavior_tree/bt_action_node.hpp"
#include "self_adaptive_behavior_tree/bt_refresh_control_node.hpp"
#include "self_adaptive_behavior_tree/bt_service_node.hpp"

using refresh_ros_msgs::action::SelfAdaptiveAction;
using refresh_ros_msgs::srv::SelfAdaptiveActionEstimate;

using BT::NodeStatus;

namespace ReFRESH_BT {
class ROS_Action_EX_Node : public BT::RosActionNode<SelfAdaptiveAction> {
 public:
  ROS_Action_EX_Node(const std::string& instance_name, const BT::NodeConfiguration& conf,
                     const BT::RosActionNodeParams& params,
                     typename std::shared_ptr<ActionClient> external_action_client = {})
      : RosActionNode<SelfAdaptiveAction>(instance_name, conf, params, external_action_client) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("action_request"), BT::InputPort<std::string>("arguments")};
  }

  bool sendGoal(Goal& goal) override;
};

class ROS_Action_EV_Node : public BT::ActionEvaluatorNode<SelfAdaptiveAction> {
 public:
  ROS_Action_EV_Node(const std::string& name, const BT::NodeConfiguration& conf)
      : BT::ActionEvaluatorNode<SelfAdaptiveAction>(name, conf) {}

  virtual NodeStatus spinOnce() override;
};

class ROS_Action_ES_Node : public BT::RosServiceNode<SelfAdaptiveActionEstimate> {
 public:
  ROS_Action_ES_Node(const std::string& instance_name, const BT::NodeConfiguration& conf,
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

}  // namespace ReFRESH_BT

#endif
