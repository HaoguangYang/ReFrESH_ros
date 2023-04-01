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

class AgeAttr : public MsgQualityAttr {
 public:
  virtual void configure(rclcpp::Node* nodeHandle, const std::string& msgType,
                         const double& tolerance, const YAML::Node& config) override {
    nh_ = nodeHandle;
    msgType_ = get_topic_type_from_string_type(msgType);
    serializer_ = std::make_unique<rclcpp::SerializationBase>(getSerializer(msgType));
    tolerance_ = tolerance;
    if (config["field"]) field_ = config["field"].as<std::string>();
    if (config["offset_field"]) offsetField_ = config["offset_field"].as<std::string>();
    if (config["offset_direction"])
      offsetDir_ = (config["offset_direction"].as<int>()) >= 0 ? 1 : -1;
    configured_ = true;
  };

  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) override {
    rclcpp::Time tnow = nh_->now();
    YAML::Node deserializedMsg = deserializeMessage(msgRaw);
    unsigned long msgStamp = getStampNanosecs(deserializedMsg);
    unsigned long age = tnow.nanoseconds() - (msgStamp == 0) ? lastActive.nanoseconds() : msgStamp;
    return {static_cast<double>(age) * 1.0e-9 / tolerance_, true};
  }

 protected:
  virtual YAML::Node findDefaultField(const YAML::Node& in) const override {
    // default fields that time information may be stored
    if (in["header"]["stamp"]) return in["header"]["stamp"];
    if (in["stamp"]) return in["stamp"];
    return in;
  }

  unsigned long getStampNanosecs(const YAML::Node& in) const {
    const unsigned long SEC_TO_NANOSEC = 1000000000UL;
    unsigned long result = 0;
    // if field is provided, use field. Otherwise, exploit proper field
    auto target = findField(in, field_);
    if (target["sec"] && target["nanosec"]) {
      result = target["sec"].as<unsigned long>() * SEC_TO_NANOSEC +
               target["nanosec"].as<unsigned long>();
    }
    // get offset value (positive)
    if (offsetField_.length()) {
      target = findField(in, offsetField_);
      if (target) {
        if (target["sec"] && target["nanosec"]) {
          result -= offsetDir_ * (target["sec"].as<unsigned long>() * SEC_TO_NANOSEC +
                                  target["nanosec"].as<unsigned long>());
        } else {
          result -= offsetDir_ * (static_cast<long int>(target.as<double>() * SEC_TO_NANOSEC));
        }
      }
    }
    return result;
  }

  std::string field_ = "";
  std::string offsetField_ = "";
  int offsetDir_ = 1;
};

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

}  // namespace ReFRESH

#endif  // AGE_ATTR_PLUGIN_HPP_
