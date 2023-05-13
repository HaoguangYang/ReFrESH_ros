#ifndef MODULAR_REDUNDANCY_NODE_HPP_
#define MODULAR_REDUNDANCY_NODE_HPP_

#include "self_awareness_attachment/refresh_self_awareness_attachment.hpp"
#include "self_awareness_attachment/visibility_control.hpp"

namespace ReFRESH {

class SELF_AWARENESS_ATTACHMENT_PUBLIC ROS_ModularRedundancyNode
    : public ROS_NodeSelfAwarenessImpl {
 public:
  explicit ROS_ModularRedundancyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 protected:
  /* this module itself contains the self awareness attachment, therefore needs to also handle the
   * activation/deactivation of self.
   */
  virtual void moduleRequestCallback(std::unique_ptr<ModuleRequest> msg) override;

  // populate and publish telemetry
  virtual void updateCallback() override;

  virtual void estimationCallback(const SelfAdaptiveModuleEstimate::Request::SharedPtr req,
                          SelfAdaptiveModuleEstimate::Response::SharedPtr res) override;

 private:
  std::vector<std::string> topicNs_;
  std::vector<bool> publish_;
  std::vector<rclcpp::GenericPublisher::SharedPtr> mrPub_;
  std::vector<rclcpp::Time> mrPubStamp_;
  std::string outputNs_;
};

}  // namespace ReFRESH

#endif  // MODULAR_REDUNDANCY_NODE_HPP_
