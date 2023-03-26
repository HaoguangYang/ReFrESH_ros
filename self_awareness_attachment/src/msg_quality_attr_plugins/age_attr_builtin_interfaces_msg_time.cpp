#include <self_awareness_attachment/msg_quality_attr_plugins/age_attr.hpp>

#include "builtin_interfaces/msg/time.hpp"

// Apply this attribute to whatever possible message types
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ReFRESH::AgeAttrTyped<builtin_interfaces::msg::Time>,
                       ReFRESH::MsgQualityAttr)
