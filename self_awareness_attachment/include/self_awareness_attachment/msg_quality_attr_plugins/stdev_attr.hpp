#ifndef STDEV_ATTR_PLUGIN_HPP_
#define STDEV_ATTR_PLUGIN_HPP_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <type_traits>

// base class
#include "self_awareness_attachment/msg_quality_attr.hpp"

// type pinning
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

// YAML deserialization
#include <yaml-cpp/yaml.h>

using namespace Eigen;

namespace ReFRESH {

class StdevAttr : public MsgQualityAttr {
 public:
  virtual void configure(rclcpp::Node* nodeHandle, const std::string& msgType,
                         const double& tolerance, const YAML::Node& config) override {
    nh_ = nodeHandle;
    msgType_ = get_topic_type_from_string_type(msgType);
    serializer_ = std::make_unique<rclcpp::SerializationBase>(getSerializer(msgType));
    tolerance_ = tolerance;
    if (config["field"]) field_ = config["field"].as<std::string>();
    if (config["dimension_mask"]) dimensionMask_ = config["dimension_mask"].as<std::vector<bool>>();
    configured_ = true;
  }

  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) override {
    (void)lastActive;
    YAML::Node deserializedMsg = deserializeMessage(msgRaw);
    YAML::Node target = findField(deserializedMsg, field_);
    if (target.IsScalar())
      return {stdevMagnitude(target.as<double>(), dimensionMask_, tolerance_), true};
    else if (target)
      return {stdevMagnitude(target.as<std::vector<double>>(), dimensionMask_, tolerance_), true};
    else
      return {BOUNDARY_ACCEPT, true};
  }

  virtual YAML::Node findDefaultField(const YAML::Node& in) const {
    if (in["variance"]) return in["variance"];
    if (in["pose"]["covariance"]) return in["pose"]["covariance"];
    if (in["twist"]["covariance"]) return in["twist"]["covariance"];
    if (in["accel"]["covariance"]) return in["accel"]["covariance"];
    if (in["position_covariance"]) return in["position_covariance"];
    if (in["magnetic_field_covariance"]) return in["magnetic_field_covariance"];
    return in["covariance"];
  }

 protected:
  static double stdevMagnitude(const double& cov, const std::vector<bool>& mask,
                               const double& tolStdev) {
    int nDimMasked = std::reduce(mask.begin(), mask.end(), 0,
                                 [](const int& a, const bool& b) { return b ? a : a + 1; });
    if (mask.size() && nDimMasked) return 0.;
    return std::sqrt(cov) / tolStdev;
  }

  static double stdevMagnitude(const std::vector<double>& cov, const std::vector<bool>& mask,
                               const double& tolStdev) {
    int nDim = static_cast<int>(std::floor(std::sqrt(cov.size())));
    int nDimMasked = std::reduce(mask.begin(), mask.end(), 0,
                                 [](const int& a, const bool& b) { return b ? a : a + 1; });
    MatrixXd covMreshaped;
    if (mask.size() && nDimMasked) {
      int remainingDim = nDim - nDimMasked;
      if (remainingDim <= 0) return 0.;
      covMreshaped = MatrixXd::Zero(remainingDim, remainingDim);
      // pack unmasked dimensions only
      int k = 0, l = 0, m = 0;
      for (int i = 0; i < nDim; i++) {
        if (mask[i]) {
          for (int j = 0; j < nDim; j++) {
            if (mask[j]) {
              covMreshaped(k, l) = cov[m];
              l++;
            }
            m++;
          }
          k++;
        } else {
          m += nDim;  // Skip entire row
        }
        l = 0;
      }
    } else {
      covMreshaped = Map<MatrixXd>(const_cast<double*>(&cov[0]), nDim, nDim);
    }
    SelfAdjointEigenSolver<MatrixXd> eig(covMreshaped);
    VectorXd e = eig.eigenvalues();
    if (e.tail(1).value() < 0.) {
      // something went wrong with this covariance matrix. Do not use.
      return REJECT;
    }
    return std::sqrt(e.tail(1).value()) / tolStdev;
  }

  std::vector<bool> dimensionMask_ = {};

  std::string field_ = "";
};

// compile-time struct: prepare to check for msg.header field
DEFINE_MEMBER_CHECKER(variance)
DEFINE_MEMBER_CHECKER(position_covariance)
DEFINE_MEMBER_CHECKER(magnetic_field_covariance)

template <class MsgT>
class StdevAttrTyped : public StdevAttr {
 public:
  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) override {
    (void)lastActive;
    MsgT msg;
    deserializeMessage<MsgT>(msgRaw, msg);
    // compile-time static type assertion and dispatch
    if constexpr (std::is_same<typename MsgT::Type,
                               geometry_msgs::msg::PoseWithCovarianceStamped::Type>::value) {
      return {stdevMagnitude({msg.pose.covariance.begin(), msg.pose.covariance.end()},
                             dimensionMask_, tolerance_),
              true};
    } else if constexpr (std::is_same<
                             typename MsgT::Type,
                             geometry_msgs::msg::TwistWithCovarianceStamped::Type>::value) {
      return {stdevMagnitude({msg.twist.covariance.begin(), msg.twist.covariance.end()},
                             dimensionMask_, tolerance_),
              true};
    } else if constexpr (std::is_same<
                             typename MsgT::Type,
                             geometry_msgs::msg::AccelWithCovarianceStamped::Type>::value) {
      return {stdevMagnitude({msg.accel.covariance.begin(), msg.accel.covariance.end()},
                             dimensionMask_, tolerance_),
              true};
    } else if constexpr (HAS_MEMBER(typename MsgT::Type, position_covariance)) {
      return {stdevMagnitude({msg.position_covariance.begin(), msg.position_covariance.end()},
                             dimensionMask_, tolerance_),
              true};
    } else if constexpr (HAS_MEMBER(typename MsgT::Type, magnetic_field_covariance)) {
      return {stdevMagnitude(
                  {msg.magnetic_field_covariance.begin(), msg.magnetic_field_covariance.end()},
                  dimensionMask_, tolerance_),
              true};
    } else if constexpr (HAS_MEMBER(typename MsgT::Type, variance)) {
      return {stdevMagnitude(msg.variance, dimensionMask_, tolerance_), true};
    } else {
      return {stdevMagnitude({msg.covariance.begin(), msg.covariance.end()}, dimensionMask_,
                             tolerance_),
              true};
    }
  }
};

// A special case for odometry
template <>
class StdevAttrTyped<nav_msgs::msg::Odometry> : public StdevAttr {
 public:
  virtual void configure(rclcpp::Node* nodeHandle, const std::string& msgType,
                         const double& tolerance, const YAML::Node& config) override {
    nh_ = nodeHandle;
    msgType_ = get_topic_type_from_string_type(msgType);
    serializer_ = std::make_unique<rclcpp::SerializationBase>(getSerializer(msgType));
    // pose fields
    tolerance_ = tolerance;
    if (config["dimension_mask"]) dimensionMask_ = config["dimension_mask"].as<std::vector<bool>>();
    // twist fields
    if (config["twist_dimension_mask"]) {
      twistMask_ = config["twist_dimension_mask"].as<std::vector<bool>>();
    } else {
      // mask out all twist components
      twistMask_.resize(6, false);
    }
    if (config["twist_stdev_tolerance"])
      twistCovTolerance_ = config["twist_stdev_tolerance"].as<double>();
    configured_ = true;
  }

  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) override {
    (void)lastActive;
    nav_msgs::msg::Odometry msg;
    deserializeMessage<nav_msgs::msg::Odometry>(msgRaw, msg);
    double poseStdevScore = stdevMagnitude({msg.pose.covariance.begin(), msg.pose.covariance.end()},
                                           dimensionMask_, tolerance_);
    if (twistCovTolerance_ == 0.) return {poseStdevScore, true};
    double twistStdevScore = stdevMagnitude(
        {msg.twist.covariance.begin(), msg.twist.covariance.end()}, twistMask_, twistCovTolerance_);
    return {std::fmax(poseStdevScore, twistStdevScore), true};
  }

 protected:
  double twistCovTolerance_ = 0.;
  std::vector<bool> twistMask_ = {};
};

// A special case for imu
template <>
class StdevAttrTyped<sensor_msgs::msg::Imu> : public StdevAttr {
 public:
  virtual void configure(rclcpp::Node* nodeHandle, const std::string& msgType,
                         const double& tolerance, const YAML::Node& config) override {
    nh_ = nodeHandle;
    msgType_ = get_topic_type_from_string_type(msgType);
    serializer_ = std::make_unique<rclcpp::SerializationBase>(getSerializer(msgType));
    // orientation fields
    tolerance_ = tolerance;
    if (config["dimension_mask"]) dimensionMask_ = config["dimension_mask"].as<std::vector<bool>>();
    // twist fields
    if (config["angular_velocity_dimension_mask"]) {
      angularVelMask_ = config["angular_velocity_dimension_mask"].as<std::vector<bool>>();
    } else {
      // mask out all angular velocity components by default
      angularVelMask_.resize(3, false);
    }
    if (config["angular_velocity_stdev_tolerance"])
      angularVelCovTolerance_ = config["angular_velocity_stdev_tolerance"].as<double>();
    // linear acceleration fields
    if (config["linear_acceleration_dimension_mask"]) {
      linearAccelMask_ = config["linear_acceleration_dimension_mask"].as<std::vector<bool>>();
    } else {
      // mask out all angular velocity components by default
      linearAccelMask_.resize(3, false);
    }
    if (config["linear_acceleration_stdev_tolerance"])
      linearAccelCovTolerance_ = config["linear_acceleration_stdev_tolerance"].as<double>();
    configured_ = true;
  }

  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) override {
    (void)lastActive;
    sensor_msgs::msg::Imu msg;
    deserializeMessage<sensor_msgs::msg::Imu>(msgRaw, msg);
    double orientationStdevScore =
        stdevMagnitude({msg.orientation_covariance.begin(), msg.orientation_covariance.end()},
                       dimensionMask_, tolerance_);
    double angularVelStdevScore = 0.;
    double linearAccelStdevScore = 0.;
    if (angularVelCovTolerance_ > 0.) {
      angularVelStdevScore = stdevMagnitude(
          {msg.angular_velocity_covariance.begin(), msg.angular_velocity_covariance.end()},
          angularVelMask_, angularVelCovTolerance_);
    }
    if (linearAccelCovTolerance_ > 0.) {
      linearAccelStdevScore = stdevMagnitude(
          {msg.linear_acceleration_covariance.begin(), msg.linear_acceleration_covariance.end()},
          linearAccelMask_, linearAccelCovTolerance_);
    }

    return {
        std::fmax(std::fmax(orientationStdevScore, angularVelStdevScore), linearAccelStdevScore),
        true};
  }

 protected:
  double angularVelCovTolerance_ = 0.;
  std::vector<bool> angularVelMask_ = {};
  double linearAccelCovTolerance_ = 0.;
  std::vector<bool> linearAccelMask_ = {};
};

}  // namespace ReFRESH

#endif  // STDEV_ATTR_PLUGIN_HPP_
