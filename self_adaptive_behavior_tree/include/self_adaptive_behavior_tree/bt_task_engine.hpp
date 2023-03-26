#ifndef BT_TASK_ENGINE_HPP_
#define BT_TASK_ENGINE_HPP_
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "builtin_interfaces/msg/time.hpp"
#include "refresh_ros_msgs/msg/module_request.hpp"
#include "refresh_ros_msgs/msg/module_telemetry.hpp"
#include "refresh_ros_msgs/srv/module_control.hpp"
#include "self_adaptive_behavior_tree/bt_refresh_control_node.hpp"
#include "self_adaptive_behavior_tree/bt_refresh_module_node.hpp"
#include "self_adaptive_behavior_tree/bt_refresh_ros_action_node.hpp"
#include "self_adaptive_behavior_tree/visibility_control.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using refresh_ros_msgs::msg::ModuleRequest;
using refresh_ros_msgs::msg::ModuleTelemetry;
using refresh_ros_msgs::srv::ModuleControl;

namespace ReFRESH_BT {

class BehaviorTreeTaskEngine {
 public:
  BehaviorTreeTaskEngine(rclcpp::Node *handle);

  BehaviorTreeTaskEngine() = delete;

  virtual ~BehaviorTreeTaskEngine() { halt(); }

  void halt() {
    tree_.rootNode()->halt();
    status_ = BT::NodeStatus::IDLE;
  }

  // control the running status of myself -- start/stop tree, re-init, etc.
  void controlCb(const ModuleControl::Request::SharedPtr req,
                 ModuleControl::Response::SharedPtr res);

  // called upon parameter updates
  rcl_interfaces::msg::SetParametersResult reconfigCb(const std::vector<rclcpp::Parameter> &config);

  // periodically tick the tree
  void spin();

 protected:
  rclcpp::Node *nh_;
  // ros::NodeHandle *nh_;

  bool terminalStateNotified_;

  std::unique_ptr<BT::PublisherZMQ> guiTracker_ = nullptr;

  BT::BehaviorTreeFactory factory_;

  BT::Tree tree_;

  BT::NodeStatus status_;

  BT::Blackboard::Ptr blackboard_;

  std::string bt_file_;

  std::chrono::microseconds sleep_us_;

  rclcpp::ServiceBase::SharedPtr controlServer_;

  rclcpp::TimerBase::SharedPtr tickTimer_;

  int64_t lastControlStamp_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr recfgServer_;

  std::unordered_map<BT::NodeStatus, ModuleTelemetry::_status_type> statusMap_ = {
      {BT::NodeStatus::IDLE, ModuleTelemetry::PRIMARY_STATE_INACTIVE},
      {BT::NodeStatus::RUNNING, ModuleTelemetry::PRIMARY_STATE_ACTIVE},
      {BT::NodeStatus::SUCCESS, ModuleTelemetry::PRIMARY_STATE_FINALIZED},
      {BT::NodeStatus::FAILURE, ModuleTelemetry::PRIMARY_STATE_UNCONFIGURED},
  };

  const std::string empty_tree_xml_ = R"(
        <root main_tree_to_execute = "MainTree">
            <BehaviorTree ID="MainTree">
                <AlwaysSuccess name="Idle"/>
            </BehaviorTree>
        </root>
        )";

 private:
  enum params {
    TICK_FREQUENCY,
    MISSION_FILE,
  };

  std::unordered_map<std::string, params> paramMap_ = {
      {"tick_frequency", TICK_FREQUENCY},
      {"mission_file", MISSION_FILE},
  };
};

/**
 * @brief Wrapper class around the functional implementation of a ROS component.
 */
class SELF_ADAPTIVE_BEHAVIOR_TREE_PUBLIC BehaviorTreeTaskEngineNode : public rclcpp::Node {
 public:
  explicit BehaviorTreeTaskEngineNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("bt_task_engine", options) {
    instantiate_engine(std::make_unique<BehaviorTreeTaskEngine>(this));
  };

 protected:
  void instantiate_engine(std::unique_ptr<BehaviorTreeTaskEngine> &&engine) noexcept {
    m_task_engine = std::forward<std::unique_ptr<BehaviorTreeTaskEngine> &&>(engine);
  }

 private:
  std::unique_ptr<BehaviorTreeTaskEngine> m_task_engine{nullptr};
};

}  // namespace ReFRESH_BT

#endif
