#ifndef FIELD_VALUE_ATTR_PLUGIN_HPP_
#define FIELD_VALUE_ATTR_PLUGIN_HPP_

// base class
#include "self_awareness_attachment/msg_quality_attr.hpp"

// YAML deserialization
#include <yaml-cpp/yaml.h>

namespace ReFRESH {

class FieldDeviationAttr : public MsgQualityAttr {
 public:
  virtual void configure(rclcpp::Node* nodeHandle, const std::string& msgType,
                         const double& tolerance, const YAML::Node& config) override {
    nh_ = nodeHandle;
    msgType_ = get_topic_type_from_string_type(msgType);
    serializer_ = std::make_unique<rclcpp::SerializationBase>(getSerializer(msgType));
    tolerance_ = tolerance;
    if (config["field"]) field_ = config["field"].as<std::string>();
    if (config["target_value"]) targetVal_ = config["target_value"].as<double>();
    if (config["negate"]) negate_ = config["negate"].as<bool>();
    configured_ = true;
  }

  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) override {
    (void)lastActive;
    YAML::Node deserializedMsg = deserializeMessage(msgRaw);
    YAML::Node target = findField(deserializedMsg, field_);
    if (target) {
      float score = std::fabs(target.as<double>() - targetVal_) / tolerance_;
      if (negate_) return {score == 0. ? REJECT : 1. / score, true};
      return {score, true};
    } else
      return {REJECT, true};
  }

  virtual YAML::Node findDefaultField(const YAML::Node& in) const { return in["data"]; }

 protected:
  std::string field_ = "";
  double targetVal_ = 0.;
  bool negate_ = false;
};

class FieldDifferenceAttr : public MsgQualityAttr {
 public:
  virtual void configure(rclcpp::Node* nodeHandle, const std::string& msgType,
                         const double& tolerance, const YAML::Node& config) override {
    nh_ = nodeHandle;
    msgType_ = get_topic_type_from_string_type(msgType);
    serializer_ = std::make_unique<rclcpp::SerializationBase>(getSerializer(msgType));
    tolerance_ = tolerance;
    if (config["field"]) field_ = config["field"].as<std::string>();
    if (config["target_value"]) targetVal_ = config["target_value"].as<double>();
    if (config["target_inclusive"]) targetInclusive_ = config["target_inclusive"].as<bool>();
    if (config["negate"]) negate_ = config["negate"].as<bool>();
    configured_ = true;
  }

  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) override {
    (void)lastActive;
    YAML::Node deserializedMsg = deserializeMessage(msgRaw);
    YAML::Node target = findField(deserializedMsg, field_);
    if (target) {
      float score;
      if (targetInclusive_) {
        if (negate_) {
          // accepts when not less than target-tolerance
          score = std::fmax(targetVal_ - target.as<double>(), 0.) / tolerance_;
        } else {
          // accepts when not greater than target+tolerance
          score = std::fmax(target.as<double>() - targetVal_, 0.) / tolerance_;
        }
      } else {
        if (negate_) {
          // accepts when less than target-tolerance
          score = tolerance_ / std::fmax(targetVal_ - target.as<double>(), __FLT_EPSILON__);
        } else {
          // accepts when greater than target+tolerance
          score = tolerance_ / std::fmax(target.as<double>() - targetVal_, __FLT_EPSILON__);
        }
      }
      return {score, true};
    } else
      return {REJECT, true};
  }

  virtual YAML::Node findDefaultField(const YAML::Node& in) const { return in["data"]; }

 protected:
  std::string field_ = "";
  double targetVal_ = 0.;
  bool targetInclusive_ = true;
  bool negate_ = false;
};

}  // namespace ReFRESH

#endif  // FIELD_VALUE_ATTR_PLUGIN_HPP_
