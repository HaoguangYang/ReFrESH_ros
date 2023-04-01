#include "self_awareness_attachment/msg_quality_attr_plugins/field_value_attr.hpp"

// Apply this attribute to whatever possible message types
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ReFRESH::FieldDeviationAttr, ReFRESH::MsgQualityAttr)
PLUGINLIB_EXPORT_CLASS(ReFRESH::FieldDifferenceAttr, ReFRESH::MsgQualityAttr)
