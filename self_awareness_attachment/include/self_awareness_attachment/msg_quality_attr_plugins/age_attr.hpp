#ifndef AGE_ATTR_PLUGIN_HPP_
#define AGE_ATTR_PLUGIN_HPP_

#include <type_traits>

// base class
#include "self_awareness_attachment/msg_quality_attr.hpp"

// type pinning
#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/header.hpp"

// YAML deserialization
#include <yaml-cpp/yaml.h>

namespace ReFRESH {

// compile-time struct: prepare to check for msg.header field
DEFINE_MEMBER_CHECKER(header)

template <class MsgT>
class AgeAttrTyped : public MsgQualityAttr {
 public:
  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) override {
    rclcpp::Time tnow = nh_->now();
    unsigned long age;
    if constexpr (std::is_same<typename MsgT::Type, builtin_interfaces::msg::Time::Type>::value ||
                  std::is_same<typename MsgT::Type, std_msgs::msg::Header::Type>::value) {
      MsgT deserializedMsg;
      deserializeMessage<MsgT>(msgRaw, deserializedMsg);
      age = tnow.nanoseconds() - getStampNanosecs(deserializedMsg);
    } else if constexpr (HAS_MEMBER(typename MsgT::Type, header)) {
      if constexpr (std::is_same<typename MsgT::_header_type, std_msgs::msg::Header::Type>::value) {
        MsgT deserializedMsg;
        deserializeMessage<MsgT>(msgRaw, deserializedMsg);
        age = tnow.nanoseconds() - getStampNanosecs(deserializedMsg);
      } else
        age = tnow.nanoseconds() - lastActive.nanoseconds();
    } else {
      age = tnow.nanoseconds() - lastActive.nanoseconds();
    }
    return {static_cast<double>(age) * 1.0e-9 / tolerance_, true};
  }

 protected:
  unsigned long getStampNanosecs(const MsgT& in) const {
    // These type traits are evaluated at compilation.
    if constexpr (std::is_same<typename MsgT::Type, builtin_interfaces::msg::Time::Type>::value)
      return static_cast<unsigned>(in.sec) * 1000000000UL + in.nanosec;
    else if constexpr (std::is_same<typename MsgT::Type, std_msgs::msg::Header::Type>::value)
      return static_cast<unsigned>(in.stamp.sec) * 1000000000UL + in.stamp.nanosec;
    else
      return static_cast<unsigned>(in.header.stamp.sec) * 1000000000UL + in.header.stamp.nanosec;
  }
};

class AgeAttr : public MsgQualityAttr {
 public:
  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) override {
    rclcpp::Time tnow = nh_->now();
    YAML::Node deserializedMsg = deserializeMessage(msgRaw);
    unsigned long msgStamp = getStampNanosecs(deserializedMsg);
    unsigned long age = tnow.nanoseconds() - (msgStamp == 0) ? lastActive.nanoseconds() : msgStamp;
    return {static_cast<double>(age) * 1.0e-9 / tolerance_, true};
  }

 protected:
  unsigned long getStampNanosecs(const YAML::Node& in) const {
    const unsigned long SEC_TO_NANOSEC = 1000000000UL;
    if (in["sec"] && in["nanosec"])
      return in["sec"].as<unsigned long>() * SEC_TO_NANOSEC + in["nanosec"].as<unsigned long>();
    else if (in["stamp"]["sec"] && in["stamp"]["nanosec"])
      return in["stamp"]["sec"].as<unsigned long>() * SEC_TO_NANOSEC +
             in["stamp"]["nanosec"].as<unsigned long>();
    else if (in["header"]["stamp"]["sec"] && in["header"]["stamp"]["nanosec"]) {
      return in["header"]["stamp"]["sec"].as<unsigned long>() * SEC_TO_NANOSEC +
             in["header"]["stamp"]["nanosec"].as<unsigned long>();
    }
    // otherwise
    return 0;
  }
};

}  // namespace ReFRESH

#endif  // AGE_ATTR_PLUGIN_HPP_
