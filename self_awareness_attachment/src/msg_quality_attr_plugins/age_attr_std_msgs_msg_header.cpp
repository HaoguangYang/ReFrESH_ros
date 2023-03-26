#include <self_awareness_attachment/msg_quality_attr_plugins/age_attr.hpp>

#include "std_msgs/msg/header.hpp"

// Apply this attribute to whatever possible message types
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ReFRESH::AgeAttrTyped<std_msgs::msg::Header>,
                       ReFRESH::MsgQualityAttr)
