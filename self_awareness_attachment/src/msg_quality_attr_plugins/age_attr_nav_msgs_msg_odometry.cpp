#include "self_awareness_attachment/msg_quality_attr_plugins/age_attr.hpp"

#include "nav_msgs/msg/odometry.hpp"

// Apply this attribute to whatever possible message types
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ReFRESH::AgeAttrTyped<nav_msgs::msg::Odometry>,
                       ReFRESH::MsgQualityAttr)
