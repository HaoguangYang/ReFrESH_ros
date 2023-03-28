#include "self_awareness_attachment/msg_quality_attr_plugins/age_attr.hpp"

#include "sensor_msgs/msg/imu.hpp"

// Apply this attribute to whatever possible message types
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ReFRESH::AgeAttrTyped<sensor_msgs::msg::Imu>, ReFRESH::MsgQualityAttr)
