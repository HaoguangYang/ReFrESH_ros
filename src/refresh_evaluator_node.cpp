#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "refresh_ros_msgs/msg/module_connectivity.hpp"
#include "refresh_ros_msgs/msg/module_cost.hpp"
#include "refresh_ros_msgs/msg/module_telemetry.hpp"

using refresh_ros_msgs::msg::ModuleConnectivity;
using refresh_ros_msgs::msg::ModuleCost;
using refresh_ros_msgs::msg::ModuleTelemetry;

namespace ReFRESH {

class MsgQualityAttr {
 public:
  virtual void initialize(const std::string& msgType, const double& tolerance) {
    auto ts_lib = rclcpp::get_typesupport_library(msgType, "rosidl_typesupport_cpp");
    auto ts_handle = rclcpp::get_typesupport_handle(msgType, "rosidl_typesupport_cpp", *ts_lib);
    serializer_ = std::make_unique<rclcpp::SerializationBase>(ts_handle);
    tolerance_ = tolerance;
    initialized_ = true;
  };

  void deserializeMessage(const rclcpp::SerializedMessage& msgRaw, void* result) const {
    serializer_->deserialize_message(&msgRaw, result);
  }

  double normalize(const double& rawResult) const { return rawResult / tolerance_; }

  /**
   * @brief Evaluate message quality attribute based on a user-specified function.
   *
   * @param msgRaw Unserialized ROS message
   * @return std::pair<double, bool> first element: cost to be returned; second element: whether the
   * returned value is normalized w.r.t. tolerance.
   */
  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw) = 0;

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
  ROS_NodeSelfAwarenessImpl(const std::string& node_name = "ros_node_ev",
                            const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node(node_name, options) {
    freq_ = this->declare_parameter("update_frequency", 10.0);
    nodeToMonitor_ = this->declare_parameter("target_node", "");
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

    telemetryPub_ =
        this->create_publisher<ModuleTelemetry>("refresh/ros_node_telemetry", TelemetryQos());

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
  // TODO: populate and publish telemetry
  void updateCallback() {
    auto pubMsg = std::make_unique<ModuleTelemetry>();
    pubMsg->stamp = this->now();
    pubMsg->module = nodeToMonitor_;
    // TODO: pubMsg->status = ...
    // populate node inter-connectivity info
    for (size_t n = 0; n < topicNames_.size(); n++) {
      if (topicDir_[n]) {
        auto pubList = this->get_publishers_info_by_topic(topicNames_[n]);
        ModuleConnectivity conn;
        conn.last_active = recvStamp_[n];
        conn.from_module = "";
        for (const auto& item : pubList) {
          if (conn.from_module.length()) conn.from_module += ", ";
          conn.from_module += item.node_name();
        }
        conn.to_module = nodeToMonitor_;
        conn.interface_name = topicNames_[n];
        conn.interface_type = topicTypes_[n];
        pubMsg->interconnect.push_back(conn);
      } else {
        auto subList = this->get_subscriptions_info_by_topic(topicNames_[n]);
        ModuleConnectivity conn;
        conn.last_active = recvStamp_[n];
        conn.from_module = nodeToMonitor_;
        conn.to_module = "";
        for (const auto& item : subList) {
          if (item.node_name() == this->get_name()) continue;
          if (conn.to_module.length()) conn.to_module += ", ";
          conn.to_module = item.node_name();
        }
        conn.interface_name = topicNames_[n];
        conn.interface_type = topicTypes_[n];
        pubMsg->interconnect.push_back(conn);
      }
    }

    // populate message quality attributes
    for (size_t n = 0; n < qAttrLib_.size(); n++) {
      std::pair<double, bool> res = qAttrLib_[n]->evaluate(msgArray_[topicRef_[n]]);
      ModuleCost cost;
      cost.connectivity_index = static_cast<int16_t>(topicRef_[n]);
      cost.tolerance = static_cast<float>(topicQualityTol_[n]);
      cost.cost = static_cast<float>(res.first);
      cost.cost_normalized = res.second;
      pubMsg->performance_cost.push_back(cost);
    }

    // pubMsg->resource_cost.push_back(...)
  };

  // frequency at which the evaluation is run
  double freq_ = 10.0;
  rclcpp::TimerBase::SharedPtr updateTimer_;

  // topic names and types to monitor. These will reflect in the interconnect field in order
  std::vector<std::string> topicNames_;
  std::vector<std::string> topicTypes_;  // in format: "std_msgs/msg/String"
  std::vector<bool> topicDir_;

  std::vector<rclcpp::Time> recvStamp_;

  // Each attribute populates one element in the performance_cost field. topicRef_ refers
  // to the zero-based index in the interconnect field, as constructed by topicNames_.
  std::vector<std::string> topicQualityAttrType_;
  std::vector<double> topicQualityTol_;
  std::vector<unsigned int> topicRef_;

  // node name to monitor
  std::string nodeToMonitor_;

  // local storage of serialized messages
  std::vector<rclcpp::SerializedMessage> msgArray_;

  // array of subscribers that monitors and receives the serialized messages
  std::vector<rclcpp::GenericSubscription::SharedPtr> subArray_;

  // array of quality attribute plugins that will be called upon publishing telemetry
  std::vector<std::shared_ptr<ReFRESH::MsgQualityAttr>> qAttrLib_;

  // publisher that reports the status of the monitored node periodically
  rclcpp::Publisher<ModuleTelemetry>::SharedPtr telemetryPub_;
};

}  // namespace ReFRESH
