#include <algorithm>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

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
    return {0., true};
  };

  // output estimator, given raw input info
  virtual std::pair<double, bool> estimateOutput(
      const std::vector<rclcpp::SerializedMessage>& msgRawArray,
      const std::vector<std::string>& msgTypeArray,
      const std::vector<rclcpp::Time>& lastActiveArray, const std::vector<bool>& msgArrayMask) {
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

class ROS_NodeSelfAwarenessImpl : public rclcpp::Node {
 public:
  explicit ROS_NodeSelfAwarenessImpl(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("ros_node_self_awareness_attachment", options) {
    freq_ = this->declare_parameter("update_frequency", 10.0);
    nodeToMonitor_ = this->declare_parameter("target_node", "test");
    topicNames_ = this->declare_parameter<std::vector<std::string>>("topic_list.names", {});
    topicTypes_ = this->declare_parameter<std::vector<std::string>>("topic_list.msg_types", {});
    topicDir_ = this->declare_parameter<std::vector<bool>>("topic_list.is_inbound_topic", {});
    topicQualityAttrType_ =
        this->declare_parameter<std::vector<std::string>>("topic_quality.attributes", {});
    topicQualityTol_ = this->declare_parameter<std::vector<double>>("topic_quality.tolerance", {});
    topicRef_ =
        this->declare_parameter<std::vector<unsigned int>>("topic_quality.topic_list_ref", {});

    if (topicNames_.size() != topicTypes_.size() || topicNames_.size() != topicDir_.size() ||
        topicNames_.size() == 0) {
      throw std::runtime_error(
          "Topic property list is ill-formed. Obtaining topic properties from within this node is "
          "not supported yet.");
    }

    if (topicQualityAttrType_.size() != topicRef_.size() ||
        topicQualityAttrType_.size() != topicQualityTol_.size()) {
      throw std::runtime_error(
          "Topic quality attribute list is ill-formed. Obtaining topic quality attribute from "
          "within this node is not supported yet.");
    }

    subArray_.reserve(topicNames_.size());
    msgArray_.resize(topicNames_.size());
    for (size_t n = 0; n < topicNames_.size(); n++) {
      recvStamp_.push_back(this->now());
      subArray_.push_back(this->create_generic_subscription(
          topicNames_[n], topicTypes_[n], rclcpp::SensorDataQoS(),
          [&](const std::shared_ptr<rclcpp::SerializedMessage>& in) {
            msgArray_[n] = *in;
            recvStamp_[n] = this->now();
          }));
    }
    resourceTelemetrySub_ = this->create_subscription<ModuleTelemetry>(
        "refresh/evaluators/resources", TelemetryQos(), [&](std::unique_ptr<ModuleTelemetry> msg) {
          if (msg->module != this->nodeToMonitor_) return;
          resourceTelemetry_ = *msg;
        });
    moduleReqSub_ = this->create_subscription<ModuleRequest>(
        "refresh/deciders/modules/" + nodeToMonitor_, rclcpp::SensorDataQoS(),
        [&](std::unique_ptr<ModuleRequest> msg) {
          // obtain target state of nodeToMonitor_ -- perform EV or ES.
          // TODO: switch exec paths: reset timer and enable ES service, or vice versa
          if (msg->module != nodeToMonitor_)
            return;
          else if (msg->request == ModuleRequest::TRANSITION_ACTIVATE) {
            estimationService_.reset();
            updateTimer_ = this->create_wall_timer(
                std::chrono::microseconds((int64_t)(1.0e+6 / freq_)),
                std::bind(&ROS_NodeSelfAwarenessImpl::updateCallback, this));
          } else if (msg->request == ModuleRequest::TRANSITION_DEACTIVATE) {
            estimationService_ = this->create_service<SelfAdaptiveModuleEstimate>(
                "refresh/estimators/" + nodeToMonitor_,
                std::bind(&ROS_NodeSelfAwarenessImpl::estimationCallback, this, _1, _2));
            updateTimer_.reset();
          }
        });
    telemetryPub_ = this->create_publisher<ModuleTelemetry>("refresh/evaluators", TelemetryQos());

    pluginlib::ClassLoader<ReFRESH::MsgQualityAttr> qAttrLoader("ReFRESH",
                                                                "ReFRESH::MsgQualityAttr");
    qAttrLib_.reserve(topicQualityAttrType_.size());
    for (size_t n = 0; n < topicQualityAttrType_.size(); n++) {
      qAttrLib_.push_back(qAttrLoader.createSharedInstance(topicQualityAttrType_[n]));
      qAttrLib_.back()->initialize(topicTypes_[topicRef_[n]], topicQualityTol_[n]);
    }

    updateTimer_ =
        this->create_wall_timer(std::chrono::microseconds((int64_t)(1.0e+6 / freq_)),
                                std::bind(&ROS_NodeSelfAwarenessImpl::updateCallback, this));
  }

 private:
  bool isTargetNodeActive() {
    // Whether the node is running or not. ROS graph may involve some lags after a node shutting
    // down, therefore it is mainly used to monitor if a node has started up or not.
    std::vector<std::string> listOfActiveNodes = this->get_node_graph_interface()->get_node_names();
    return (std::find(listOfActiveNodes.begin(), listOfActiveNodes.end(), nodeToMonitor_) !=
            listOfActiveNodes.end());
  };

  ModuleConnectivity reportConnectivity(const bool& isInbound, const std::string& topicName,
                                        const std::string& topicType,
                                        const rclcpp::Time& lastActive) {
    ModuleConnectivity ret;
    if (isInbound) {
      // incoming messages, get publishers (nodes other than nodeToMonitor_)
      auto pubList = this->get_publishers_info_by_topic(topicName);
      ret.last_active = lastActive;
      ret.from_module = "";
      for (const auto& item : pubList) {
        if (ret.from_module.length()) ret.from_module += ", ";
        ret.from_module += item.node_name();
      }
      ret.to_module = nodeToMonitor_;
      ret.interface_name = topicName;
      ret.interface_type = topicType;
    } else {
      // outgoing messages, get subscribers other than this self awareness node.
      auto subList = this->get_subscriptions_info_by_topic(topicName);
      ret.last_active = lastActive;
      ret.from_module = nodeToMonitor_;
      ret.to_module = "";
      for (const auto& item : subList) {
        if (item.node_name() == this->get_name()) continue;
        if (ret.to_module.length()) ret.to_module += ", ";
        ret.to_module = item.node_name();
      }
      ret.interface_name = topicName;
      ret.interface_type = topicType;
    }
    return ret;
  }

  ModuleCost reportPerformanceCost(const std::shared_ptr<ReFRESH::MsgQualityAttr>& attrClass,
                                   const rclcpp::SerializedMessage& rawMsg,
                                   const unsigned int& index, const rclcpp::Time& lastActive,
                                   const double& performanceTolerance) {
    ModuleCost cost;
    std::pair<double, bool> res = attrClass->evaluate(rawMsg, lastActive);
    cost.connectivity_index = static_cast<int16_t>(index);
    cost.tolerance = static_cast<float>(performanceTolerance);
    cost.cost = static_cast<float>(res.first);
    cost.cost_normalized = res.second;
    return cost;
  }

  // populate and publish telemetry
  void updateCallback() {
    auto pubMsg = std::make_unique<ModuleTelemetry>();
    pubMsg->stamp = this->now();
    pubMsg->module = nodeToMonitor_;
    pubMsg->status = isTargetNodeActive() ? ModuleTelemetry::PRIMARY_STATE_ACTIVE
                                          : ModuleTelemetry::PRIMARY_STATE_INACTIVE;
    // populate node inter-connectivity info
    for (size_t n = 0; n < topicNames_.size(); n++) {
      pubMsg->interconnect.push_back(
          reportConnectivity(topicDir_[n], topicNames_[n], topicTypes_[n], recvStamp_[n]));
    }

    // populate message quality attributes
    for (size_t n = 0; n < qAttrLib_.size(); n++) {
      pubMsg->performance_cost.push_back(reportPerformanceCost(
          qAttrLib_[n], msgArray_[topicRef_[n]], topicRef_[n], recvStamp_[n], topicQualityTol_[n]));
    }

    // populate resource quality attributes from the received resource utilization telemetry
    size_t resourceInterconnectOffset = pubMsg->interconnect.size();
    pubMsg->interconnect.insert(pubMsg->interconnect.end(), resourceTelemetry_.interconnect.begin(),
                                resourceTelemetry_.interconnect.end());
    pubMsg->resource_cost = resourceTelemetry_.resource_cost;
    for (auto& rCost : pubMsg->resource_cost) {
      rCost.connectivity_index += resourceInterconnectOffset;
    }

    telemetryPub_->publish(std::move(pubMsg));
  };

  void estimationCallback(const SelfAdaptiveModuleEstimate::Request::SharedPtr req,
                          SelfAdaptiveModuleEstimate::Response::SharedPtr res) {
    if (req->module != nodeToMonitor_) {
      res->explanation = "Invalid request: target node name mismatch.";
      return;
    }
    res->estimate.stamp = this->now();
    res->estimate.module = this->nodeToMonitor_;
    res->estimate.status = isTargetNodeActive() ? ModuleTelemetry::PRIMARY_STATE_ACTIVE
                                                : ModuleTelemetry::PRIMARY_STATE_INACTIVE;
    // populate node inter-connectivity info
    for (size_t n = 0; n < topicNames_.size(); n++) {
      res->estimate.interconnect.push_back(
          reportConnectivity(topicDir_[n], topicNames_[n], topicTypes_[n], recvStamp_[n]));
    }
    // populate message quality attributes
    for (size_t n = 0; n < qAttrLib_.size(); n++) {
      if (topicDir_[topicRef_[n]]) {
        // inbound message, assume they are available upon estimation (incremental reconfig)
        res->estimate.performance_cost.push_back(
            reportPerformanceCost(qAttrLib_[n], msgArray_[topicRef_[n]], topicRef_[n],
                                  recvStamp_[n], topicQualityTol_[n]));
      } else {
        // call the estimateOutput method instead
        ModuleCost cost;
        std::pair<double, bool> est =
            qAttrLib_[n]->estimateOutput(msgArray_, topicTypes_, recvStamp_, topicDir_);
        cost.connectivity_index = static_cast<int16_t>(topicRef_[n]);
        cost.tolerance = static_cast<float>(topicQualityTol_[n]);
        cost.cost = static_cast<float>(est.first);
        cost.cost_normalized = est.second;
        res->estimate.performance_cost.push_back(cost);
      }
    }
    // populate resource quality attributes from the received resource utilization telemetry
    size_t resourceInterconnectOffset = res->estimate.interconnect.size();
    res->estimate.interconnect.insert(res->estimate.interconnect.end(),
                                      resourceTelemetry_.interconnect.begin(),
                                      resourceTelemetry_.interconnect.end());
    res->estimate.resource_cost = resourceTelemetry_.resource_cost;
    for (auto& rCost : res->estimate.resource_cost) {
      rCost.connectivity_index += resourceInterconnectOffset;
    }
  };

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
  std::vector<unsigned int> topicRef_;

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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ReFRESH::ROS_NodeSelfAwarenessImpl)
