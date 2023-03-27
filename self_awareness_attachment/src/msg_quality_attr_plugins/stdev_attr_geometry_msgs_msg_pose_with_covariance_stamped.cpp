#include "self_awareness_attachment/msg_quality_attr_plugins/stdev_attr.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// Apply this attribute to whatever possible message types
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ReFRESH::StdevAttrTyped<geometry_msgs::msg::PoseWithCovarianceStamped>,
                       ReFRESH::MsgQualityAttr)
