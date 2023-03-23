#ifndef REFRESH_EVALUATOR_NODE_HPP_
#define REFRESH_EVALUATOR_NODE_HPP_

#include <algorithm>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "refresh_ros2/visibility_control.hpp"
#include "refresh_ros_msgs/msg/module_connectivity.hpp"
#include "refresh_ros_msgs/msg/module_cost.hpp"
#include "refresh_ros_msgs/msg/module_request.hpp"
#include "refresh_ros_msgs/msg/module_telemetry.hpp"
#include "refresh_ros_msgs/srv/self_adaptive_module_estimate.hpp"

using refresh_ros_msgs::msg::ModuleConnectivity;
using refresh_ros_msgs::msg::ModuleCost;
using refresh_ros_msgs::msg::ModuleRequest;
using refresh_ros_msgs::msg::ModuleTelemetry;
using refresh_ros_msgs::srv::SelfAdaptiveModuleEstimate;
using std::placeholders::_1;
using std::placeholders::_2;

namespace ReFRESH {

class MsgQualityAttr {
 public:
  virtual void initialize(const std::string& msgType, const double& tolerance) {
    serializer_ = std::make_unique<rclcpp::SerializationBase>(getSerializer(msgType));
    tolerance_ = tolerance;
    initialized_ = true;
  };

  static rclcpp::SerializationBase getSerializer(const std::string& msgType) {
    auto ts_lib = rclcpp::get_typesupport_library(msgType, "rosidl_typesupport_cpp");
    auto ts_handle = rclcpp::get_typesupport_handle(msgType, "rosidl_typesupport_cpp", *ts_lib);
    return rclcpp::SerializationBase(ts_handle);
  }

  void deserializeMessage(const rclcpp::SerializedMessage& msgRaw, void* result) const {
    serializer_->deserialize_message(&msgRaw, result);
  }

  static void deserializeMessage(const rclcpp::SerializedMessage& msgRaw,
                                 const rclcpp::SerializationBase& serializer, void* result) {
    serializer.deserialize_message(&msgRaw, result);
  }

  static void deserializeMessage(const rclcpp::SerializedMessage& msgRaw,
                                 const std::string& msgType, void* result) {
    getSerializer(msgType).deserialize_message(&msgRaw, result);
  }

  double normalize(const double& rawResult) const { return rawResult / tolerance_; }

  /**
   * @brief Evaluate message quality attribute based on a user-specified function.
   *
   * @param msgRaw Unserialized ROS message
   * @param lastActive Timestamp of receiving the unserialized ROS message
   * @return std::pair<double, bool> first element: cost to be returned; second element: whether the
   * returned value is normalized w.r.t. tolerance.
   */
  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) {
    (void)msgRaw;
    (void)lastActive;
    return {0., true};
  };

  /**
   * @brief Estimator specific to output topics, based on inputs from prescribed topics.
   *
   * @param msgRawArray All input and output messages
   * @param msgTypeArray All input and output message types
   * @param lastActiveArray Vector of timestamps that the messages are last active
   * @param msgArrayMask Mask for incoming messages
   * @return std::pair<double, bool> first element: cost to be returned; second element: whether the
   * returned value is normalized w.r.t. tolerance.
   */
  virtual std::pair<double, bool> estimateOutput(
      const std::vector<rclcpp::SerializedMessage>& msgRawArray,
      const std::vector<std::string>& msgTypeArray,
      const std::vector<rclcpp::Time>& lastActiveArray, const std::vector<bool>& msgArrayMask) {
    (void)msgRawArray;
    (void)msgTypeArray;
    (void)lastActiveArray;
    (void)msgArrayMask;
    return {0., true};
  };

  virtual ~MsgQualityAttr(){};

 protected:
  MsgQualityAttr(){};
  bool initialized_ = false;
  double tolerance_ = 1.0;
  std::unique_ptr<rclcpp::SerializationBase> serializer_ = nullptr;
};

class TelemetryQos : public rclcpp::QoS {
 public:
  explicit TelemetryQos() : rclcpp::QoS(200){};
};

class REFRESH_ROS2_PUBLIC ROS_NodeSelfAwarenessImpl : public rclcpp::Node {
 public:
  explicit ROS_NodeSelfAwarenessImpl(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void moduleRequestCallback(std::unique_ptr<ModuleRequest> msg);

  bool isTargetNodeActive() {
    // Whether the node is running or not. ROS graph may involve some lags after a node shutting
    // down, therefore it is mainly used to monitor if a node has started up or not.
    std::vector<std::string> listOfActiveNodes = this->get_node_graph_interface()->get_node_names();
    return (std::find(listOfActiveNodes.begin(), listOfActiveNodes.end(), nodeToMonitor_) !=
            listOfActiveNodes.end());
  };

  ModuleConnectivity reportConnectivity(const bool& isInbound, const std::string& topicName,
                                        const std::string& topicType,
                                        const rclcpp::Time& lastActive);

  ModuleCost reportPerformanceCost(const std::shared_ptr<ReFRESH::MsgQualityAttr>& attrClass,
                                   const rclcpp::SerializedMessage& rawMsg,
                                   const unsigned int& index, const rclcpp::Time& lastActive,
                                   const double& performanceTolerance);

  // populate and publish telemetry
  void updateCallback();

  void estimationCallback(const SelfAdaptiveModuleEstimate::Request::SharedPtr req,
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

#endif  // REFRESH_EVALUATOR_NODE_HPP_
