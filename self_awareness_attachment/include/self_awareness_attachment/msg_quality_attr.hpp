#ifndef MSG_QUALITY_ATTR_HPP_
#define MSG_QUALITY_ATTR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

// compile-time evaluation of certain types
#include <type_traits>

// deserialize into YAML formats
#include <yaml-cpp/yaml.h>

// dynamic message introspection
#include "dynmsg/message_reading.hpp"

namespace ReFRESH {

class MsgQualityAttr {
 public:
  virtual void configure(rclcpp::Node* nodeHandle, const std::string& msgType,
                         const double& tolerance, const YAML::Node& config) {
    nh_ = nodeHandle;
    msgType_ = get_topic_type_from_string_type(msgType);
    serializer_ = std::make_unique<rclcpp::SerializationBase>(getSerializer(msgType));
    tolerance_ = tolerance;
    configured_ = true;
    (void)config;
  };

  static rclcpp::SerializationBase getSerializer(const std::string& msgType) {
    auto ts_lib = rclcpp::get_typesupport_library(msgType, "rosidl_typesupport_cpp");
    auto ts_handle = rclcpp::get_typesupport_handle(msgType, "rosidl_typesupport_cpp", *ts_lib);
    return rclcpp::SerializationBase(ts_handle);
  }

  static InterfaceTypeName get_topic_type_from_string_type(const std::string& type) {
    std::string::size_type split_at = type.find('/');
    if (split_at == std::string::npos) {
      throw std::runtime_error("invalid type specification");
    }
    return InterfaceTypeName(type.substr(0, split_at), type.substr(split_at + 1));
  }

  static YAML::Node deserializeMessage(const rclcpp::SerializedMessage& msgRaw,
                                       const std::string& msgType) {
    RosMessage message;
    InterfaceTypeName interface_type = get_topic_type_from_string_type(msgType);
    dynmsg::c::ros_message_init(interface_type, &message);
    getSerializer(msgType).deserialize_message(&msgRaw, (void*)&(message.data[0]));
    return dynmsg::c::message_to_yaml(message);
  }

  YAML::Node deserializeMessage(const rclcpp::SerializedMessage& msgRaw) const {
    RosMessage message;
    dynmsg::c::ros_message_init(msgType_, &message);
    serializer_->deserialize_message(&msgRaw, (void*)&(message.data[0]));
    return dynmsg::c::message_to_yaml(message);
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

  template <class MsgT>
  void deserializeMessage(const rclcpp::SerializedMessage& msgRaw, MsgT& result) {
    serializer_->deserialize_message(&msgRaw, (void*)&result);
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

  rclcpp::Node* nh_;
  InterfaceTypeName msgType_;
  bool configured_ = false;
  double tolerance_ = 1.0;
  std::unique_ptr<rclcpp::SerializationBase> serializer_ = nullptr;
};

// helper macro to determine if a class has a specific member
#define DEFINE_MEMBER_CHECKER(member)                                                            \
  template <typename T, typename V = bool>                                                       \
  struct has_##member : std::false_type {};                                                      \
  template <typename T>                                                                          \
  struct has_##member<                                                                           \
      T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().member), void>::value, \
                                 bool>::type> : std::true_type {};

#define HAS_MEMBER(C, member) has_##member<C>::value

}  // namespace ReFRESH

#endif  // MSG_QUALITY_ATTR_HPP_
