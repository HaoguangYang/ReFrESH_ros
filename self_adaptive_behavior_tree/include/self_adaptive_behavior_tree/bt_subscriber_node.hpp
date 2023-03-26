#ifndef BEHAVIOR_TREE_ROS2__BT_SUBSCRIBER_NODE_HPP_
#define BEHAVIOR_TREE_ROS2__BT_SUBSCRIBER_NODE_HPP_

#include <memory>
#include <string>

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace BT {

struct RosSubscriberNodeParams {
  std::shared_ptr<rclcpp::Node> nh;
  std::string topic_name;
  rclcpp::QoS qos;

  RosSubscriberNodeParams(const std::shared_ptr<rclcpp::Node>& node,
                          const std::string& topic_name = "",
                          const rclcpp::QoS& qos = rclcpp::SensorDataQoS())
      : nh(node), topic_name(topic_name), qos(qos){};
};

/**
 * @brief Abstract class representing use to subscribe to a ROS 2 topic.
 *
 * It will try to be non-blocking for the entire duration of the call.
 * The derived class whould reimplement the virtual methods as described below.
 */
template <class MessageT>
class RosSubscriberNode : public StatefulActionNode {
 public:
  // Type definitions
  using MessageType = MessageT;
  using Params = RosSubscriberNodeParams;
  using SubscriberClient = rclcpp::SubscriptionBase;

  /** You are not supposed to instantiate this class directly, the factory will do it.
   * To register this class into the factory, use:
   *
   *    RegisterRosSubscriber<DerivedClasss>(factory, params)
   *
   * Note that if the external_subscriber is not set, the constructor will build its own.
   * */
  explicit RosSubscriberNode(const std::string& instance_name, const NodeConfiguration& conf,
                             const RosSubscriberNodeParams& params,
                             typename std::shared_ptr<SubscriberClient> external_subscriber = {});

  virtual ~RosSubscriberNode() = default;

  /**
   * @brief These ports will be added automatically if this Node is registered using
   * RegisterRosAction<DeriveClass>()
   *
   * @return PortsList
   */
  static PortsList providedPorts() {
    return {InputPort<std::string>("topic_name", "", "name of the ROS topic"),
            InputPort<unsigned>("buffer_length", 0, "length of message buffer")};
  }

  /**
   * @brief Methods to be implemented by the user.
   *
   * @return true The node will exit with result isFailure().
   * @return false The node will keep running.
   */
  virtual bool isExitCondition() = 0;

  /**
   * @brief When exiting the node, what state it is in.
   *
   * @return true The node exits with NodeStatus::FAILURE.
   * @return false The node exits with NodeStatus::SUCCESS.
   */
  virtual bool isFailure() = 0;

  /**
   * @brief The method to be called upon receiving a new message from the topic.
   *
   * @param msg Message the subsciber received.
   */
  void onMessageCb(std::unique_ptr<MessageType> msg) override;

  NodeStatus onStart() override;

  NodeStatus onRunning() override;

  void onHalted() override;

 protected:
  std::shared_ptr<rclcpp::Node> node_;
  const std::string topic_name_;
  const rclcpp::QoS qos_;

  MessageT message_;

 private:
  typename std::shared_ptr<SubscriberClient> subscriber_;
};

/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT>
static void RegisterRosSubscriber(
    BehaviorTreeFactory& factory, const std::string& registration_ID,
    const RosSubscriberNodeParams& params,
    std::shared_ptr<typename DerivedT::ServiceClient> external_client = {}) {
  NodeBuilder builder = [=](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(name, config, params, external_client);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = DerivedT::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class T>
inline RosSubscriberNode<T>::RosSubscriberNode(
    const std::string& instance_name, const NodeConfiguration& conf,
    const RosSubscriberNodeParams& params,
    typename std::shared_ptr<SubscriberClient> external_subscriber)
    : StatefulActionNode(instance_name, conf),
      node_(params.nh),
      topic_name_(params.topic_name),
      qos_(params.qos) {
  if (external_subscriber) {
    subscriber_ = external_subscriber;
  } else {
    subscriber_ = nullptr;
  }
};

template <class T>
inline NodeStatus RosSubscriberNode<T>::onStart() {
  if (subscriber_ == nullptr) {
    Result inRes;
    // clean inputs -- if port override exists, use them.
    // Otherwise, use default params passed in at initialization.
    std::string topic_name = "";
    if (!(inRes = getInput<std::string>("topic_name", topic_name))) topic_name = topic_name_;
    if (topic_name.length() == 0) topic_name = topic_name_;
    unsigned buffer_len = 0;
    if (!(inRes = getInput<unsigned>("buffer_length", buffer_len))) buffer_len = qos_.depth;
    if (buffer_len == 0) buffer_len = qos_.depth;
    rclcpp::QoS qos(qos_);
    qos.depth = buffer_len;

    subscriber_ = node_->create_subscription<T>(
        topic_name, qos,
        std::bind(&RosSubscriberNode<T>::onMessageCb, this, std::placeholders::_1));
  }
  return NodeStatus::RUNNING;
};

template <class T>
inline void RosSubscriberNode<T>::onMessageCb(std::unique_ptr<T> msg) {
  message_ = *msg;
};

template <class T>
inline NodeStatus RosSubscriberNode<T>::onRunning() {
  if (!isExitCondition()) return NodeStatus::RUNNING;
  subscriber_.reset();
  if (isFailure()) {
    return NodeStatus::FAILURE;
  }
  return NodeStatus::SUCCESS;
}

template <class T>
inline void RosSubscriberNode<T>::onHalted() {
  subscriber_.reset();
  setStatus(NodeStatus::IDLE);
}

}  // namespace BT

#endif  // BEHAVIOR_TREE_ROS2__BT_SUBSCRIBER_NODE_HPP_
