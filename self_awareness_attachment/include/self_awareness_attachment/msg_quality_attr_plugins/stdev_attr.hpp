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
    if (config["dimension_mask"]) dimensionMask_ = config["dimension_mask"].as<std::vector<bool>>();
    configured_ = true;
  }

  virtual std::pair<double, bool> evaluate(const rclcpp::SerializedMessage& msgRaw,
                                           const rclcpp::Time& lastActive) override {
    (void)lastActive;
    YAML::Node deserializedMsg = deserializeMessage(msgRaw);
    if (deserializedMsg["covariance"]) {
      auto v = deserializedMsg["covariance"].as<std::vector<double>>();
      return {stdevMagnitude(v, tolerance_), true};
    } else if (deserializedMsg["variance"]) {
      return {stdevMagnitude(deserializedMsg["variance"].as<double>(), tolerance_), true};
    } else if (deserializedMsg["pose"] && deserializedMsg["pose"]["covariance"]) {
      auto v = deserializedMsg["pose"]["covariance"].as<std::vector<double>>();
      return {stdevMagnitude(v, tolerance_), true};
    } else if (deserializedMsg["twist"] && deserializedMsg["twist"]["covariance"]) {
      auto v = deserializedMsg["twist"]["covariance"].as<std::vector<double>>();
      return {stdevMagnitude(v, tolerance_), true};
    } else if (deserializedMsg["accel"] && deserializedMsg["accel"]["covariance"]) {
      auto v = deserializedMsg["accel"]["covariance"].as<std::vector<double>>();
      return {stdevMagnitude(v, tolerance_), true};
    } else if (deserializedMsg["position_covariance"]) {
      auto v = deserializedMsg["position_covariance"].as<std::vector<double>>();
      return {stdevMagnitude(v, tolerance_), true};
    } else if (deserializedMsg["magnetic_field_covariance"]) {
      auto v = deserializedMsg["magnetic_field_covariance"].as<std::vector<double>>();
      return {stdevMagnitude(v, tolerance_), true};
    }
    return {BOUNDARY_ACCEPT, true};
  }

 protected:
  double stdevMagnitude(const double& cov, const double& tolStdev) const {
    int nDimMasked = std::accumulate(dimensionMask_.begin(), dimensionMask_.end(), 0,
                                     [](const int& a, const bool& b) { return b ? a + 1 : a; });
    if (dimensionMask_.size() && nDimMasked) return 0.;
    return std::sqrt(cov) / tolStdev;
  }

  double stdevMagnitude(const std::vector<double>& cov, const double& tolStdev) const {
    const double* ptr = &cov[0];
    int nDim = static_cast<int>(std::floor(std::sqrt(cov.size())));
    int nDimMasked = std::accumulate(dimensionMask_.begin(), dimensionMask_.end(), 0,
                                     [](const int& a, const bool& b) { return b ? a + 1 : a; });
    MatrixXd covMreshaped;
    if (dimensionMask_.size() && nDimMasked) {
      covMreshaped = MatrixXd::Zero(nDim - nDimMasked, nDim - nDimMasked);
      // pack unmasked dimensions only
      int k = 0, l = 0, m = 0;
      for (int i = 0; i < nDim; i++) {
        if (!dimensionMask_[i]) {
          for (int j = 0; j < nDim; j++) {
            if (!dimensionMask_[j]) {
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
      covMreshaped = Map<MatrixXd>(const_cast<double*>(ptr), nDim, nDim);
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
      return {stdevMagnitude({msg.pose.covariance.begin(), msg.pose.covariance.end()}, tolerance_),
              true};
    } else if constexpr (std::is_same<
                             typename MsgT::Type,
                             geometry_msgs::msg::TwistWithCovarianceStamped::Type>::value) {
      return {
          stdevMagnitude({msg.twist.covariance.begin(), msg.twist.covariance.end()}, tolerance_),
          true};
    } else if constexpr (std::is_same<
                             typename MsgT::Type,
                             geometry_msgs::msg::AccelWithCovarianceStamped::Type>::value) {
      return {
          stdevMagnitude({msg.accel.covariance.begin(), msg.accel.covariance.end()}, tolerance_),
          true};
    } else if constexpr (HAS_MEMBER(typename MsgT::Type, position_covariance)) {
      return {stdevMagnitude({msg.position_covariance.begin(), msg.position_covariance.end()},
                             tolerance_),
              true};
    } else if constexpr (HAS_MEMBER(typename MsgT::Type, magnetic_field_covariance)) {
      return {stdevMagnitude(
                  {msg.magnetic_field_covariance.begin(), msg.magnetic_field_covariance.end()},
                  tolerance_),
              true};
    } else if constexpr (HAS_MEMBER(typename MsgT::Type, variance)) {
      return {stdevMagnitude(msg.variance, tolerance_), true};
    } else {
      return {stdevMagnitude({msg.covariance.begin(), msg.covariance.end()}, tolerance_), true};
    }
  }
};

}  // namespace ReFRESH

#include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(ReFRESH::StdevAttr<nav_msgs::msg::Odometry>, ReFRESH::MsgQualityAttr)

#endif  // STDEV_ATTR_PLUGIN_HPP_