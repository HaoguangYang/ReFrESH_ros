#include "self_awareness_attachment/modular_redundancy.hpp"

namespace ReFRESH {

ROS_ModularRedundancyNode::ROS_ModularRedundancyNode(const rclcpp::NodeOptions& options)
    : ROS_NodeSelfAwarenessImpl(options) {
  // frequency at which the modular redundancy decision is run
  freq_ = this->declare_parameter("update_frequency", 10.0);
  nodeToMonitor_ = this->get_name();
  topicNs_ = this->declare_parameter("topic_namespaces", std::vector<std::string>{});
  topicNames_ = this->declare_parameter("topic_list.names", std::vector<std::string>{});
  topicTypes_ = this->declare_parameter("topic_list.msg_types", std::vector<std::string>{});
  // only in-bound topics are provided
  topicDir_ = std::vector<bool>(topicNames_.size(), true);
  topicQualityAttrType_ =
      this->declare_parameter("topic_quality.attributes", std::vector<std::string>{});
  topicQualityTol_ = this->declare_parameter("topic_quality.tolerance", std::vector<double>{});
  // vector of YAML-formatted string that passes in additional configurations for each attribute.
  topicQualityAttrCfg_ =
      this->declare_parameter("topic_quality.config", std::vector<std::string>{});
  topicRef_ = this->declare_parameter("topic_quality.topic_list_ref", std::vector<int64_t>{});
  outputNs_ = this->declare_parameter("output_topic_namespace", "modular_redundancy_repub");

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

  for (size_t n = 0; n < topicNames_.size(); n++) {
    mrPub_.push_back(this->create_generic_publisher(outputNs_ + topicNames_[n], topicTypes_[n],
                                                    rclcpp::SystemDefaultsQoS()));
    mrPubStamp_.push_back(this->now());
  }
  publish_.resize(topicNs_.size() * topicNames_.size(), false);

  // identical subscription into each provided namespace
  subArray_.reserve(topicNs_.size() * topicNames_.size());
  msgArray_.resize(topicNs_.size() * topicNames_.size());
  for (size_t i = 0; i < topicNs_.size(); i++) {
    const size_t offset = i * topicNames_.size();
    for (size_t n = 0; n < topicNames_.size(); n++) {
      recvStamp_.push_back(this->now());
      subArray_.push_back(this->create_generic_subscription(
          topicNames_[n], topicTypes_[n], rclcpp::SensorDataQoS(),
          // Lambda function for publishing the relayed message based on masks
          [&](const std::shared_ptr<rclcpp::SerializedMessage>& in) {
            msgArray_[offset + n] = *in;
            recvStamp_[offset + n] = this->now();
            if (publish_[offset + n]) mrPub_[n]->publish(*in);
          }));
    }
  }

  resourceTelemetrySub_ = this->create_subscription<ModuleTelemetry>(
      "refresh/evaluators/resources", TelemetryQos(), [&](std::unique_ptr<ModuleTelemetry> msg) {
        if (msg->module != nodeToMonitor_) return;
        resourceTelemetry_ = *msg;
      });

  moduleReqSub_ = this->create_subscription<ModuleRequest>(
      "refresh/deciders/modules/" + nodeToMonitor_, rclcpp::SensorDataQoS(),
      std::bind(&ROS_ModularRedundancyNode::moduleRequestCallback, this, _1));

  telemetryPub_ = this->create_publisher<ModuleTelemetry>("refresh/evaluators", TelemetryQos());

  pluginlib::ClassLoader<ReFRESH::MsgQualityAttr> qAttrLoader("ReFRESH", "ReFRESH::MsgQualityAttr");
  qAttrLib_.reserve(topicNs_.size() * topicQualityAttrType_.size());
  for (size_t i = 0; i < topicNs_.size(); i++) {
    for (size_t n = 0; n < topicQualityAttrType_.size(); n++) {
      qAttrLib_.push_back(qAttrLoader.createSharedInstance(topicQualityAttrType_[n]));
      qAttrLib_.back()->configure(this, topicTypes_[topicRef_[n]], topicQualityTol_[n],
                                  YAML::Load(topicQualityAttrCfg_[n]));
    }
  }

  updateTimer_ =
      this->create_wall_timer(std::chrono::microseconds((int64_t)(1.0e+6 / freq_)),
                              std::bind(&ROS_ModularRedundancyNode::updateCallback, this));
}

void ROS_ModularRedundancyNode::moduleRequestCallback(std::unique_ptr<ModuleRequest> msg) {
  // obtain target state of nodeToMonitor_ -- perform EV or ES.
  if (msg->module != nodeToMonitor_)
    return;
  else if (msg->request == ModuleRequest::TRANSITION_ACTIVATE) {
    // switch exec paths: reset timer and enable ES service, or vice versa
    estimationService_.reset();
    updateTimer_ =
        this->create_wall_timer(std::chrono::microseconds((int64_t)(1.0e+6 / freq_)),
                                std::bind(&ROS_ModularRedundancyNode::updateCallback, this));
    // updateCallback will enable publishing of output.
  } else if (msg->request == ModuleRequest::TRANSITION_DEACTIVATE) {
    estimationService_ = this->create_service<SelfAdaptiveModuleEstimate>(
        "refresh/estimators/" + nodeToMonitor_,
        std::bind(&ROS_ModularRedundancyNode::estimationCallback, this, _1, _2));
    updateTimer_.reset();
    // need to disable publishing of output
    publish_.resize(topicNs_.size() * topicNames_.size(), false);
  }
}

// populate and publish telemetry
void ROS_ModularRedundancyNode::updateCallback() {
  auto pubMsg = std::make_unique<ModuleTelemetry>();
  pubMsg->stamp = this->now();
  pubMsg->module = nodeToMonitor_;
  pubMsg->status = isTargetNodeActive() ? ModuleTelemetry::PRIMARY_STATE_ACTIVE
                                        : ModuleTelemetry::PRIMARY_STATE_INACTIVE;
  // populate node inter-connectivity info -- input (topicDir_ has input info only)
  for (size_t i = 0; i < topicNs_.size(); i++) {
    size_t base = i * topicNames_.size();
    for (size_t n = 0; n < topicNames_.size(); n++) {
      pubMsg->interconnect.push_back(reportConnectivity(true, topicNs_[i] + topicNames_[n],
                                                        topicTypes_[n], recvStamp_[base + n]));
    }
  }

  // populate node inter-connectivity info -- output
  for (size_t n = 0; n < topicNames_.size(); n++) {
    pubMsg->interconnect.push_back(
        reportConnectivity(false, outputNs_ + topicNames_[n], topicTypes_[n], mrPubStamp_[n]));
  }

  // populate message quality attributes for inputs
  for (size_t i = 0; i < topicNs_.size(); i++) {
    for (size_t j = 0; j < topicQualityAttrType_.size(); j++) {
      // N.B. topicRef_[n_wrapped] is per namespace. Need to extend to the entire subscribed topics
      // later.
      pubMsg->performance_cost.push_back(
          reportPerformanceCost(qAttrLib_[i], msgArray_[topicRef_[j]], topicRef_[j],
                                recvStamp_[topicRef_[j]], topicQualityTol_[j]));
    }
  }

  // Based on message quality attributes, determine publishing mask
  std::vector<size_t> best_ind(topicNames_.size());
  std::vector<float> best_quality_attr_normalized(topicNames_.size());
  // circle through each cluster of subscribed topics
  for (size_t i = 0; i < topicNs_.size(); i++) {
    // circle through each attribute of the topic cluster
    for (size_t j = 0; j < topicQualityAttrType_.size(); j++) {
      size_t cost_ind = i * topicQualityAttrType_.size() + j;
      auto cost = &(pubMsg->performance_cost[cost_ind]);
      // N.B. topicRef_[n_wrapped] (cost.connectivity_index) is still per namespace
      size_t topic_ind = cost->connectivity_index;
      // if cost_normalized not there, calculate it.
      float cost_norm = cost->cost_normalized ? cost->cost : cost->cost / cost->tolerance;
      if (cost->cost_normalized < best_quality_attr_normalized[topic_ind]) {
        best_ind[topic_ind] = i;
        best_quality_attr_normalized[topic_ind] = cost_norm;
      }
    }
  }
  for (size_t i = 0; i < topicNs_.size(); i++) {
    for (size_t j = 0; j < topicNames_.size(); j++) {
      publish_[i * topicNames_.size() + j] =
          best_ind[j] == i;  // && best_quality_attr_normalized[j] <= BOUNDARY_ACCEPT;
    }
  }

  // N.B. topicRef_[n_wrapped] (cost.connectivity_index) is still per namespace. Fix it here.
  size_t ind = 0;
  for (size_t i = 0; i < topicNs_.size(); i++) {
    size_t offset = i * topicNames_.size();
    for (size_t j = 0; j < topicQualityAttrType_.size(); j++)
      pubMsg->performance_cost[ind++].connectivity_index += offset;
  }
  // Based on publishing mask, determine message quality attributes for outputs
  for (size_t i = 0; i < topicQualityAttrType_.size(); i++) {
    size_t attr_ind = best_ind[topicRef_[i]] * topicQualityAttrType_.size() + i;
    pubMsg->performance_cost.push_back(pubMsg->performance_cost[attr_ind]);
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

void ROS_ModularRedundancyNode::estimationCallback(
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
RCLCPP_COMPONENTS_REGISTER_NODE(ReFRESH::ROS_ModularRedundancyNode)
