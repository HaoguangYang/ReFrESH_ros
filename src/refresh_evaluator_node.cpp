#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "refresh_ros_msgs/msg/module_telemetry.hpp"

using refresh_ros_msgs::msg::ModuleTelemetry;

namespace ReFRESH {

class MsgQualityAttr {
 public:
  virtual void initialize(const std::string& msgType) {
    auto ts_lib = rclcpp::get_typesupport_library(msgType, "rosidl_typesupport_cpp");
    auto ts_handle = rclcpp::get_typesupport_handle(msgType, "rosidl_typesupport_cpp", *ts_lib);
    serializer_ = std::make_unique<rclcpp::SerializationBase>(ts_handle);
  };

  virtual float evaluate(const rclcpp::SerializedMessage& msgRaw) {
    // YourMsgType msgOut;
    // serializer_->deserialize_message(&msgRaw, &msgOut);
    // perform further processing
    return 0.;
  };

  virtual ~MsgQualityAttr(){};

 protected:
  MsgQualityAttr(){};

 private:
  std::unique_ptr<rclcpp::SerializationBase> serializer_ = nullptr;
};

class ROS_NodeEvaluatorNode : public rclcpp::Node {
 public:
  ROS_NodeEvaluatorNode(const std::string& node_name,
                        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node(node_name, options) {
    nodeToMonitor_ = this->declare_parameter("target_node", "");
    topicNames_ = this->declare_parameter<std::vector<std::string>>("topic_list.names", {});
    topicTypes_ = this->declare_parameter<std::vector<std::string>>("topic_list.msg_types", {});
    topicDir_ = this->declare_parameter<std::vector<bool>>("topic_list.is_inbound_topic", {});
    topicQualityAttrType_ =
        this->declare_parameter<std::vector<std::string>>("topic_quality.attributes", {});
    topicRef_ =
        this->declare_parameter<std::vector<unsigned int>>("topic_quality.topic_list_ref", {});

    if (topicNames_.size() != topicTypes_.size() || topicNames_.size() != topicDir_.size() ||
        topicNames_.size() == 0) {
      throw std::runtime_error(
          "Topic property list is ill-formed. Obtaining topic properties from within this node is "
          "not supported yet.");
    }

    if (topicQualityAttrType_.size() != topicRef_.size()) {
      throw std::runtime_error(
          "Topic quality attribute list is ill-formed. Obtaining topic quality attribute from "
          "within this node is not supported yet.");
    }

    subArray_.reserve(topicNames_.size());
    msgArray_.resize(topicNames_.size());
    for (size_t n = 0; n < topicNames_.size(); n++) {
      subArray_.push_back(this->create_generic_subscription(
          topicNames_[n], topicTypes_[n], rclcpp::SensorDataQoS(),
          [&](const std::shared_ptr<rclcpp::SerializedMessage>& in) { msgArray_[n] = *in; }));
    }

    pluginlib::ClassLoader<ReFRESH::MsgQualityAttr> qAttrLoader("ReFRESH",
                                                                "ReFRESH::MsgQualityAttr");
    qAttrLib_.reserve(topicQualityAttrType_.size());
    for (size_t n = 0; n < topicQualityAttrType_.size(); n++) {
      qAttrLib_.push_back(qAttrLoader.createSharedInstance(topicQualityAttrType_[n]));
      qAttrLib_.back()->initialize(topicTypes_[topicRef_[n]]);
    }
  }

 private:
  // TODO: populate and publish telemetry

  // topic names and types to monitor. These will reflect in the interconnect field in order
  std::vector<std::string> topicNames_;
  std::vector<std::string> topicTypes_;  // in format: "std_msgs/msg/String"
  std::vector<bool> topicDir_;

  // Each attribute populates one element in the performance_cost field. topicRef_ refers
  // to the zero-based index in the interconnect field, as constructed by topicNames_.
  std::vector<std::string> topicQualityAttrType_;
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
  rclcpp::Publisher<ModuleTelemetry>::SharedPtr pub_;
};

}  // namespace ReFRESH
