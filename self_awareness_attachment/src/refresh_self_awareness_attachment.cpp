#include "self_awareness_attachment/refresh_self_awareness_attachment.hpp"

namespace ReFRESH {

ROS_NodeSelfAwarenessImpl::ROS_NodeSelfAwarenessImpl(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ros_node_self_awareness_attachment", options) {
  freq_ = this->declare_parameter("update_frequency", 10.0);
  nodeToMonitor_ = this->declare_parameter("target_node", "test");
  topicNames_ = this->declare_parameter("topic_list.names", std::vector<std::string>{});
  topicTypes_ = this->declare_parameter("topic_list.msg_types", std::vector<std::string>{});
  topicDir_ = this->declare_parameter("topic_list.is_inbound_topic", std::vector<bool>{});
  topicQualityAttrType_ =
      this->declare_parameter("topic_quality.attributes", std::vector<std::string>{});
  topicQualityTol_ = this->declare_parameter("topic_quality.tolerance", std::vector<double>{});
  // vector of YAML-formatted string that passes in additional configurations for each attribute.
  topicQualityAttrCfg_ =
      this->declare_parameter("topic_quality.config", std::vector<std::string>{});
  topicRef_ = this->declare_parameter("topic_quality.topic_list_ref", std::vector<int64_t>{});

  if (topicNames_.size() != topicTypes_.size() || topicNames_.size() != topicDir_.size() ||
      topicNames_.size() == 0) {
    throw std::runtime_error(
        "Topic property list is ill-formed. Obtaining topic properties from within this node is "
        "not supported yet.");
  }

  if (topicQualityAttrType_.size() != topicRef_.size() ||
      topicQualityAttrType_.size() != topicQualityAttrCfg_.size()) {
    throw std::runtime_error(
        "Topic quality attribute list is ill-formed. Obtaining topic quality attribute from "
        "within this node is not supported yet.");
  }

  subArray_.reserve(topicNames_.size());
  msgArray_.resize(topicNames_.size());
  for (size_t n = 0; n < topicNames_.size(); n++) {
    recvStamp_.push_back(this->now());
    subArray_.push_back(this->create_generic_subscription(
        topicNames_[n], topicTypes_[n], rclcpp::SensorDataQoS(),
        [&](const std::shared_ptr<rclcpp::SerializedMessage>& in) {
          msgArray_[n] = *in;
          recvStamp_[n] = this->now();
        }));
  }
  resourceTelemetrySub_ = this->create_subscription<ModuleTelemetry>(
      "refresh/evaluators/resources", TelemetryQos(), [&](std::unique_ptr<ModuleTelemetry> msg) {
        if (msg->module != this->nodeToMonitor_) return;
        resourceTelemetry_ = *msg;
      });
  moduleReqSub_ = this->create_subscription<ModuleRequest>(
      "refresh/deciders/modules/" + nodeToMonitor_, rclcpp::SensorDataQoS(),
      std::bind(&ROS_NodeSelfAwarenessImpl::moduleRequestCallback, this, _1));
  telemetryPub_ = this->create_publisher<ModuleTelemetry>("refresh/evaluators", TelemetryQos());

  pluginlib::ClassLoader<ReFRESH::MsgQualityAttr> qAttrLoader("ReFRESH", "ReFRESH::MsgQualityAttr");
  qAttrLib_.reserve(topicQualityAttrType_.size());
  for (size_t n = 0; n < topicQualityAttrType_.size(); n++) {
    qAttrLib_.push_back(qAttrLoader.createSharedInstance(topicQualityAttrType_[n]));
    qAttrLib_.back()->configure(this, topicTypes_[topicRef_[n]], topicQualityTol_[n],
                                YAML::Load(topicQualityAttrCfg_[n]));
  }

  updateTimer_ =
      this->create_wall_timer(std::chrono::microseconds((int64_t)(1.0e+6 / freq_)),
                              std::bind(&ROS_NodeSelfAwarenessImpl::updateCallback, this));
}

void ROS_NodeSelfAwarenessImpl::moduleRequestCallback(std::unique_ptr<ModuleRequest> msg) {
  // obtain target state of nodeToMonitor_ -- perform EV or ES.
  if (msg->module != nodeToMonitor_)
    return;
  else if (msg->request == ModuleRequest::TRANSITION_ACTIVATE) {
    // switch exec paths: reset timer and enable ES service, or vice versa
    estimationService_.reset();
    updateTimer_ =
        this->create_wall_timer(std::chrono::microseconds((int64_t)(1.0e+6 / freq_)),
                                std::bind(&ROS_NodeSelfAwarenessImpl::updateCallback, this));
  } else if (msg->request == ModuleRequest::TRANSITION_DEACTIVATE) {
    estimationService_ = this->create_service<SelfAdaptiveModuleEstimate>(
        "refresh/estimators/" + nodeToMonitor_,
        std::bind(&ROS_NodeSelfAwarenessImpl::estimationCallback, this, _1, _2));
    updateTimer_.reset();
  }
}

ModuleConnectivity ROS_NodeSelfAwarenessImpl::reportConnectivity(const bool& isInbound,
                                                                 const std::string& topicName,
                                                                 const std::string& topicType,
                                                                 const rclcpp::Time& lastActive) {
  ModuleConnectivity ret;
  if (isInbound) {
    // incoming messages, get publishers (nodes other than nodeToMonitor_)
    auto pubList = this->get_publishers_info_by_topic(topicName);
    ret.last_active = lastActive;
    ret.from_module = "";
    for (const auto& item : pubList) {
      if (ret.from_module.length()) ret.from_module += ", ";
      ret.from_module += item.node_name();
    }
    ret.to_module = nodeToMonitor_;
    ret.interface_name = topicName;
    ret.interface_type = topicType;
  } else {
    // outgoing messages, get subscribers other than this self awareness node.
    auto subList = this->get_subscriptions_info_by_topic(topicName);
    ret.last_active = lastActive;
    ret.from_module = nodeToMonitor_;
    ret.to_module = "";
    for (const auto& item : subList) {
      if (item.node_name() == this->get_name()) continue;
      if (ret.to_module.length()) ret.to_module += ", ";
      ret.to_module = item.node_name();
    }
    ret.interface_name = topicName;
    ret.interface_type = topicType;
  }
  return ret;
}

ModuleCost ROS_NodeSelfAwarenessImpl::reportPerformanceCost(
    const std::shared_ptr<ReFRESH::MsgQualityAttr>& attrClass,
    const rclcpp::SerializedMessage& rawMsg, const unsigned int& index,
    const rclcpp::Time& lastActive, const double& performanceTolerance) {
  ModuleCost cost;
  std::pair<double, bool> res = attrClass->evaluate(rawMsg, lastActive);
  cost.connectivity_index = static_cast<int16_t>(index);
  cost.tolerance = static_cast<float>(performanceTolerance);
  cost.cost = static_cast<float>(res.first);
  cost.cost_normalized = res.second;
  return cost;
}

// populate and publish telemetry
void ROS_NodeSelfAwarenessImpl::updateCallback() {
  auto pubMsg = std::make_unique<ModuleTelemetry>();
  pubMsg->stamp = this->now();
  pubMsg->module = nodeToMonitor_;
  pubMsg->status = isTargetNodeActive() ? ModuleTelemetry::PRIMARY_STATE_ACTIVE
                                        : ModuleTelemetry::PRIMARY_STATE_INACTIVE;
  // populate node inter-connectivity info
  for (size_t n = 0; n < topicNames_.size(); n++) {
    pubMsg->interconnect.push_back(
        reportConnectivity(topicDir_[n], topicNames_[n], topicTypes_[n], recvStamp_[n]));
  }

  // populate message quality attributes
  for (size_t n = 0; n < qAttrLib_.size(); n++) {
    pubMsg->performance_cost.push_back(reportPerformanceCost(
        qAttrLib_[n], msgArray_[topicRef_[n]], topicRef_[n], recvStamp_[n], topicQualityTol_[n]));
  }

  // populate resource quality attributes from the received resource utilization telemetry
  size_t resourceInterconnectOffset = pubMsg->interconnect.size();
  pubMsg->interconnect.insert(pubMsg->interconnect.end(), resourceTelemetry_.interconnect.begin(),
                              resourceTelemetry_.interconnect.end());
  pubMsg->resource_cost = resourceTelemetry_.resource_cost;
  for (auto& rCost : pubMsg->resource_cost) {
    rCost.connectivity_index += resourceInterconnectOffset;
  }

  telemetryPub_->publish(std::move(pubMsg));
}

void ROS_NodeSelfAwarenessImpl::estimationCallback(
    const SelfAdaptiveModuleEstimate::Request::SharedPtr req,
    SelfAdaptiveModuleEstimate::Response::SharedPtr res) {
  if (req->module != nodeToMonitor_) {
    res->explanation = "Invalid request: target node name mismatch.";
    return;
  }
  res->estimate.stamp = this->now();
  res->estimate.module = this->nodeToMonitor_;
  res->estimate.status = isTargetNodeActive() ? ModuleTelemetry::PRIMARY_STATE_ACTIVE
                                              : ModuleTelemetry::PRIMARY_STATE_INACTIVE;
  // populate node inter-connectivity info
  for (size_t n = 0; n < topicNames_.size(); n++) {
    res->estimate.interconnect.push_back(
        reportConnectivity(topicDir_[n], topicNames_[n], topicTypes_[n], recvStamp_[n]));
  }
  // populate message quality attributes
  for (size_t n = 0; n < qAttrLib_.size(); n++) {
    if (topicDir_[topicRef_[n]]) {
      // inbound message, assume they are available upon estimation (incremental reconfig)
      res->estimate.performance_cost.push_back(reportPerformanceCost(
          qAttrLib_[n], msgArray_[topicRef_[n]], topicRef_[n], recvStamp_[n], topicQualityTol_[n]));
    } else {
      // call the estimateOutput method instead
      ModuleCost cost;
      std::pair<double, bool> est =
          qAttrLib_[n]->estimateOutput(msgArray_, topicTypes_, recvStamp_, topicDir_);
      cost.connectivity_index = static_cast<int16_t>(topicRef_[n]);
      cost.tolerance = static_cast<float>(topicQualityTol_[n]);
      cost.cost = static_cast<float>(est.first);
      cost.cost_normalized = est.second;
      res->estimate.performance_cost.push_back(cost);
    }
  }
  // populate resource quality attributes from the received resource utilization telemetry
  size_t resourceInterconnectOffset = res->estimate.interconnect.size();
  res->estimate.interconnect.insert(res->estimate.interconnect.end(),
                                    resourceTelemetry_.interconnect.begin(),
                                    resourceTelemetry_.interconnect.end());
  res->estimate.resource_cost = resourceTelemetry_.resource_cost;
  for (auto& rCost : res->estimate.resource_cost) {
    rCost.connectivity_index += resourceInterconnectOffset;
  }
}

}  // namespace ReFRESH

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ReFRESH::ROS_NodeSelfAwarenessImpl)
