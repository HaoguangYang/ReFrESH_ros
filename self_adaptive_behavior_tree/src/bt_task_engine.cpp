#include "self_adaptive_behavior_tree/bt_task_engine.hpp"

namespace ReFRESH_BT {

BehaviorTreeTaskEngine::BehaviorTreeTaskEngine(rclcpp::Node *handle) : nh_(handle) {
  // Behaviortree utilities
  blackboard_ = BT::Blackboard::create();
  status_ = BT::NodeStatus::IDLE;
  terminalStateNotified_ = false;
  tree_ = factory_.createTreeFromText(empty_tree_xml_, blackboard_);
  guiTracker_ = std::make_unique<BT::PublisherZMQ>(tree_);
  bt_file_ = handle->declare_parameter<std::string>("mission_file", "");
  sleep_us_ = std::chrono::microseconds(
      (unsigned int)(1000000 / handle->declare_parameter<double>("tick_frequency", 10.0)));

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
  tickTimer_ = handle->create_wall_timer(sleep_us_, std::bind(&BehaviorTreeTaskEngine::spin, this));

  // Register nodes
  BT::RegisterRosAction<ROS_Action_EX_Node>(
      factory_, "ReFRESH_ROS_Action_EX",
      ROS_Action_EX_Node::Params(std::shared_ptr<rclcpp::Node>(handle),
                                 "refresh_ros_ex_action_test", 500));
  BT::RegisterActionEvaluator<ROS_Action_EV_Node>(factory_, "ReFRESH_ROS_Action_EV");
  BT::RegisterRosService<ROS_Action_ES_Node>(
      factory_, "ReFRESH_ROS_Action_ES",
      ROS_Action_ES_Node::Params(std::shared_ptr<rclcpp::Node>(handle),
                                 "refresh_ros_es_service_test", 100));
  factory_.registerNodeType<ReFRESH_Module>("ReFRESH_Module");
  factory_.registerNodeType<DeciderNode>("ReFRESH_Decider");
  factory_.registerNodeType<ReactorNode>("ReFRESH_Reactor");
  // Derived task engine class can use this base initializer and register more custom nodes
}

void BehaviorTreeTaskEngine::controlCb(const ModuleControl::Request::SharedPtr req,
                                       ModuleControl::Response::SharedPtr res) {
  int64_t controlStamp_ = req->request.stamp.nanosec + req->request.stamp.sec * 1000000000L;
  if (controlStamp_ < lastControlStamp_ || req->request.module != nh_->get_name()) {
    // do nothing
    res->status.stamp = nh_->now();
    res->status.module = nh_->get_name();
    res->status.status = statusMap_[tree_.rootNode()->status()];
  }
  lastControlStamp_ = controlStamp_;
  switch (req->request.request) {
    case ModuleRequest::TRANSITION_CONFIGURE:
      std::cout << "Configure Behavior Tree using mission file: " << bt_file_ << std::endl;
      // if invalid tree / file, use an empty tree.
      try {
        tree_ = factory_.createTreeFromFile(bt_file_, blackboard_);
      } catch (...) {
        std::cout << "Error occured creating tree from file: " << bt_file_
                  << "! Using an EMPTY TREE instead." << std::endl;
        tree_ = factory_.createTreeFromText(empty_tree_xml_, blackboard_);
      }
      guiTracker_ = std::make_unique<BT::PublisherZMQ>(tree_);
      terminalStateNotified_ = false;
      status_ = BT::NodeStatus::IDLE;
      break;

    case ModuleRequest::TRANSITION_ACTIVATE:
      terminalStateNotified_ = false;
      status_ = tree_.tickRoot();
      break;

    case ModuleRequest::TRANSITION_DEACTIVATE:
      halt();
      status_ = BT::NodeStatus::IDLE;
      break;

    case ModuleRequest::TRANSITION_UNCONFIGURED_SHUTDOWN:
    case ModuleRequest::TRANSITION_INACTIVE_SHUTDOWN:
    case ModuleRequest::TRANSITION_ACTIVE_SHUTDOWN:
      halt();
      status_ = BT::NodeStatus::SUCCESS;
      break;

    case ModuleRequest::TRANSITION_CLEANUP:
      halt();
      status_ = BT::NodeStatus::FAILURE;
      break;

    default:
      // do nothing
      break;
  }
  res->status.stamp = nh_->now();
  res->status.module = nh_->get_name();
  res->status.status = statusMap_[tree_.rootNode()->status()];
}

rcl_interfaces::msg::SetParametersResult BehaviorTreeTaskEngine::reconfigCb(
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
        guiTracker_ = std::make_unique<BT::PublisherZMQ>(tree_);
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

void BehaviorTreeTaskEngine::spin() {
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

}  // namespace ReFRESH_BT

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ReFRESH_BT::BehaviorTreeTaskEngineNode)
