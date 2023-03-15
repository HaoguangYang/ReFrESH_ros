#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "builtin_interfaces/msg/time.hpp"
#include "refresh_ros2/bt_refresh_control_node.hpp"
#include "refresh_ros2/bt_refresh_module_node.hpp"
#include "refresh_ros2/bt_refresh_ros_action_node.hpp"
#include "refresh_ros2/visibility_control.hpp"
#include "refresh_ros_msgs/msg/module_request.hpp"
#include "refresh_ros_msgs/msg/module_telemetry.hpp"
#include "refresh_ros_msgs/srv/module_control.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using refresh_ros_msgs::msg::ModuleRequest;
using refresh_ros_msgs::msg::ModuleTelemetry;
using refresh_ros_msgs::srv::ModuleControl;

namespace ReFRESH {

class BehaviorTreeTaskEngine {
 public:
  BehaviorTreeTaskEngine(rclcpp::Node *handle) : nh_(handle), sleep_us_(100000UL) {
    // Register nodes
    BT::RegisterRosAction<ReFRESH_ROS_EX_node>(
        factory_, "ReFRESH_ROS_EX",
        ReFRESH_ROS_EX_node::Params(std::shared_ptr<rclcpp::Node>(handle),
                                    "refresh_ros_ex_action_test", 500));
    BT::RegisterActionEvaluator<ReFRESH_ROS_EV_node>(factory_, "ReFRESH_ROS_EV");
    BT::RegisterRosService<ReFrESH_ROS_ES_node>(
        factory_, "ReFRESH_ROS_ES",
        ReFrESH_ROS_ES_node::Params(std::shared_ptr<rclcpp::Node>(handle),
                                    "refresh_ros_es_service_test", 100));
    factory_.registerNodeType<ReFRESH_Module>("ReFRESH_Module");
    factory_.registerNodeType<ReFRESH_Decider>("ReFRESH_Decider");
    factory_.registerNodeType<ReFRESH_Reactor>("ReFRESH_Reactor");

    // Behaviortree utilities
    blackboard_ = BT::Blackboard::create();
    status_ = BT::NodeStatus::RUNNING;
    terminalStateNotified_ = false;
    tree_ = factory_.createTreeFromText(empty_tree_xml_, blackboard_);
    guiTracker_ = new BT::PublisherZMQ(tree_);
    bt_file_ = "";

    // ROS utilities
    // Controlling service for this node
    std::string nodeName = handle->get_name();
    controlServer_ = handle->create_service<ModuleControl>(
        "refresh/" + nodeName + "/module_control",
        std::bind(&BehaviorTreeTaskEngine::controlCb, this, _1, _2));
    // parameter callback (dynamic reconfigure for ROS1)
    // dynamic_reconfigure::Server<refresh_ros::BehaviorTreeTaskEngineConfig>::CallbackType recfgCb_
    //   = boost::bind(&BehaviorTreeTaskEngine::reconfigCb, this, ::_1, ::_2);
    // recfgServer_.setCallback(recfgCb_);
    recfgServer_ = handle->add_on_set_parameters_callback(
        std::bind(&BehaviorTreeTaskEngine::reconfigCb, this, _1));

    // timer for cyclic ticks
    tickTimer_ =
        handle->create_wall_timer(sleep_us_, std::bind(&BehaviorTreeTaskEngine::spin, this));
  }

  BehaviorTreeTaskEngine() = delete;

  virtual ~BehaviorTreeTaskEngine() {
    halt();
    delete (guiTracker_);
  }

  inline void halt() {
    tree_.rootNode()->halt();
    status_ = BT::NodeStatus::IDLE;
  }

  static inline int64_t toNSec(const builtin_interfaces::msg::Time &time) {
    return time.nanosec + time.sec * 1000000000L;
  }

  void controlCb(const ModuleControl::Request::SharedPtr req,
                 ModuleControl::Response::SharedPtr res) {
    int64_t controlStamp_ = toNSec(req->request.stamp);
    if (controlStamp_ < lastControlStamp_ || req->request.module != nh_->get_name()) {
      // do nothing
      res->status.stamp = nh_->now();
      res->status.module = nh_->get_name();
      res->status.status = statusMap_[tree_.rootNode()->status()];
    }
    lastControlStamp_ = controlStamp_;
    switch (req->request.request) {
      case ModuleRequest::SPAWN:
      case ModuleRequest::ON:
        status_ = BT::NodeStatus::RUNNING;
        break;

      case ModuleRequest::WAKEUP:
        status_ = tree_.tickRoot();
        break;

      case ModuleRequest::OFF:
      case ModuleRequest::TERM:
      case ModuleRequest::KILL:
        halt();
        break;

      case ModuleRequest::CLEAR:
        status_ = BT::NodeStatus::IDLE;
        break;

      case ModuleRequest::REINIT:
        std::cout << "Reinitializing mission file: " << bt_file_ << std::endl;
        halt();
        // if invalid tree / file, use an empty tree.
        try {
          tree_ = factory_.createTreeFromFile(bt_file_, blackboard_);
        } catch (...) {
          std::cout << "Error occured creating tree from file: " << bt_file_
                    << "! Using an EMPTY TREE instead." << std::endl;
          tree_ = factory_.createTreeFromText(empty_tree_xml_, blackboard_);
        }
        status_ = BT::NodeStatus::RUNNING;
        delete (guiTracker_);
        guiTracker_ = new BT::PublisherZMQ(tree_);
        break;

      default:
        // do nothing
        break;
    }
    res->status.stamp = nh_->now();
    res->status.module = nh_->get_name();
    res->status.status = statusMap_[tree_.rootNode()->status()];
  }

  rcl_interfaces::msg::SetParametersResult reconfigCb(
      const std::vector<rclcpp::Parameter> &config) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    std::string tmp;
    for (const auto &param : config) {
      switch (paramMap_[param.get_name()]) {
        case MISSION_FILE:
          tmp = param.as_string();
          if (tmp != bt_file_) bt_file_ = tmp;
          // needs to rebuild the behavior tree
          std::cout << "Using mission file: " << bt_file_ << std::endl;
          halt();
          // if invalid tree / file, use an empty tree.
          try {
            tree_ = factory_.createTreeFromFile(bt_file_, blackboard_);
          } catch (...) {
            result.successful = false;
            result.reason = "Error occured creating tree from file: " + bt_file_ +
                            "! Using an EMPTY TREE instead.";
            tree_ = factory_.createTreeFromText(empty_tree_xml_, blackboard_);
          }
          terminalStateNotified_ = false;
          status_ = BT::NodeStatus::RUNNING;
          delete (guiTracker_);
          guiTracker_ = new BT::PublisherZMQ(tree_);
          break;
        case TICK_FREQUENCY:
          sleep_us_ = std::chrono::microseconds((unsigned int)(1000000 / param.as_double()));
          break;
        default:
          break;
      }
    }
    return result;
  }

  void spin() {
    // the behavior tree engine should not stop until external management node exits.
    // while (ros::ok()) {
    //   tree_.sleep(sleep_us_);
    //   ros::spinOnce();
    switch (status_) {
      case BT::NodeStatus::SUCCESS:
        if (!terminalStateNotified_) {
          RCLCPP_INFO(nh_->get_logger(), "Task completed with status SUCCESS.");
          terminalStateNotified_ = true;
        }
        break;

      case BT::NodeStatus::FAILURE:
        if (!terminalStateNotified_) {
          RCLCPP_INFO(nh_->get_logger(), "Task completed with status FAILURE.");
          terminalStateNotified_ = true;
        }
        break;

      case BT::NodeStatus::RUNNING:
        status_ = tree_.tickRoot();
        break;

      // IDLE
      default:
        break;
    }
    // }
    // halt();
  }

 protected:
  rclcpp::Node *nh_;
  // ros::NodeHandle *nh_;

  bool terminalStateNotified_;

  BT::PublisherZMQ *guiTracker_;

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
      {BT::NodeStatus::IDLE, ModuleTelemetry::READY},
      {BT::NodeStatus::RUNNING, ModuleTelemetry::RUNNING},
      {BT::NodeStatus::SUCCESS, ModuleTelemetry::OFF},
      {BT::NodeStatus::FAILURE, ModuleTelemetry::ERROR},
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
class REFRESH_TASK_ENGINE_PUBLIC BehaviorTreeTaskEngineNode : public rclcpp::Node {
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

}  // namespace ReFrESH

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ReFRESH::BehaviorTreeTaskEngineNode)
