#ifndef SELF_AWARENESS_ATTACHMENT_NODE_HPP_
#define SELF_AWARENESS_ATTACHMENT_NODE_HPP_

#include <algorithm>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "refresh_ros_msgs/msg/module_connectivity.hpp"
#include "refresh_ros_msgs/msg/module_cost.hpp"
#include "refresh_ros_msgs/msg/module_request.hpp"
#include "refresh_ros_msgs/msg/module_telemetry.hpp"
#include "refresh_ros_msgs/srv/self_adaptive_module_estimate.hpp"

// in-package headers
#include "self_awareness_attachment/msg_quality_attr.hpp"
#include "self_awareness_attachment/visibility_control.hpp"

// YAML loader from string
#include <yaml-cpp/yaml.h>

using std::placeholders::_1;
using std::placeholders::_2;

using refresh_ros_msgs::msg::ModuleConnectivity;
using refresh_ros_msgs::msg::ModuleCost;
using refresh_ros_msgs::msg::ModuleRequest;
using refresh_ros_msgs::msg::ModuleTelemetry;
using refresh_ros_msgs::srv::SelfAdaptiveModuleEstimate;

namespace ReFRESH {

class TelemetryQos : public rclcpp::QoS {
 public:
  explicit TelemetryQos() : rclcpp::QoS(200){};
};

class SELF_AWARENESS_ATTACHMENT_PUBLIC ROS_NodeSelfAwarenessImpl : public rclcpp::Node {
 public:
  explicit ROS_NodeSelfAwarenessImpl(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 protected:
  virtual void moduleRequestCallback(std::unique_ptr<ModuleRequest> msg);

  bool isTargetNodeActive() {
    // Whether the node is running or not. ROS graph may involve some lags after a node shutting
    // down, therefore it is mainly used to monitor if a node has started up or not.
    std::vector<std::string> listOfActiveNodes = this->get_node_graph_interface()->get_node_names();
    return (std::find(listOfActiveNodes.begin(), listOfActiveNodes.end(), nodeToMonitor_) !=
            listOfActiveNodes.end());
  };

  ModuleConnectivity reportConnectivity(const bool& isInbound, const std::string& topicName,
                                        const std::string& topicType,
                                        const rclcpp::Time& lastActive) const;

  ModuleCost reportPerformanceCost(std::shared_ptr<ReFRESH::MsgQualityAttr>& attrClass,
                                   const rclcpp::SerializedMessage& rawMsg,
                                   const unsigned int& index, const rclcpp::Time& lastActive,
                                   const double& performanceTolerance);

  // populate and publish telemetry
  virtual void updateCallback();

  virtual void estimationCallback(const SelfAdaptiveModuleEstimate::Request::SharedPtr req,
                          SelfAdaptiveModuleEstimate::Response::SharedPtr res);

  // frequency at which the evaluation is run
  double freq_ = 10.0;
  rclcpp::TimerBase::SharedPtr updateTimer_;

  // topic names and types to monitor. These will reflect in the interconnect field in order
  std::vector<std::string> topicNames_;
  std::vector<std::string> topicTypes_;  // in format: "std_msgs/msg/String"
  std::vector<bool> topicDir_;

  // Each attribute populates one element in the performance_cost field. topicRef_ refers
  // to the zero-based index in the interconnect field, as constructed by topicNames_.
  std::vector<std::string> topicQualityAttrType_;
  std::vector<double> topicQualityTol_;
  std::vector<std::string> topicQualityAttrCfg_;
  std::vector<int64_t> topicRef_;

  // node name to monitor
  std::string nodeToMonitor_;

  // local storage of serialized messages and update stamp
  std::vector<rclcpp::SerializedMessage> msgArray_;
  std::vector<rclcpp::Time> recvStamp_;

  // array of subscribers that monitors and receives the serialized messages
  std::vector<rclcpp::GenericSubscription::SharedPtr> subArray_;

  // array of quality attribute plugins that will be called upon publishing telemetry
  std::vector<std::shared_ptr<ReFRESH::MsgQualityAttr>> qAttrLib_;

  // subscriber to resource utilization info
  rclcpp::SubscriptionBase::SharedPtr resourceTelemetrySub_;
  ModuleTelemetry resourceTelemetry_;

  // subscriber to target node control message (dispatched)
  rclcpp::SubscriptionBase::SharedPtr moduleReqSub_;
  bool targetModuleActive_;

  // Estimatr service called when the nodeToMonitor_ is inactive
  rclcpp::ServiceBase::SharedPtr estimationService_;

  // publisher that reports the status of the monitored node periodically
  rclcpp::Publisher<ModuleTelemetry>::SharedPtr telemetryPub_;
};

}  // namespace ReFRESH

#endif  // SELF_AWARENESS_ATTACHMENT_NODE_HPP_
