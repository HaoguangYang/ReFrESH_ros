// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREE_BT_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_BT_ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

namespace BT
{

enum ActionNodeErrorCode{
  SERVER_UNREACHABLE,
  SEND_GOAL_TIMEOUT,
  GOAL_REJECTED_BY_SERVER,
  ACTION_ABORTED,
  ACTION_CANCELLED,
  INVALID_GOAL
};

/** Helper Node to call an actionlib::SimpleActionClient<>
 * inside a BT::ActionNode.
 *
 * Note that the user must implement the methods:
 *
 *  - sendGoal
 *  - onResult
 *  - onFailure
 *  - halt (optionally)
 *
 */
template<class ActionT>
class RosActionNode : public BT::ActionNodeBase
{
protected:

  RosActionNode(ros::NodeHandle& nh, const std::string& name,
                const BT::NodeConfiguration & conf):
    BT::ActionNodeBase(name, conf), node_(nh)
  {}

public:

  using ActionType = ActionT;
  using ActionClient = actionlib::SimpleActionClient<ActionT>;
  using BaseClass = RosActionNode<ActionT>;
  using Goal  = typename ActionT::_action_goal_type::_goal_type;
  // using Result = typename ActionT::_action_result_type::_result_type;
  using WrappedResult = typename
                        ActionT::_action_result_type::_result_type::ConstPtr;
  using Feedback = typename
                        ActionT::_action_feedback_type::_feedback_type;
  using FeedbackConstPtr = typename
                        ActionT::_action_feedback_type::_feedback_type::ConstPtr;

  RosActionNode() = delete;

  virtual ~RosActionNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosAction<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "name of the Action Server"),
      InputPort<unsigned>("timeout", 500, "timeout to connect (milliseconds)"),
      OutputPort<Feedback>("feedback", "pointer to action feedback")
    };
  }

  /// Method called when the Action makes a transition from IDLE to RUNNING.
  /// If it return false, the entire action is immediately aborted, it returns
  /// FAILURE and no request is sent to the server.
  virtual bool sendGoal(Goal& goal) = 0;

  virtual std::unique_ptr<Feedback> resultToFeedback(const WrappedResult& res)
  {
    return nullptr;
  }

  /// Method (to be implemented by the user) to receive the reply.
  /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
  virtual NodeStatus onResult( const WrappedResult& result )
  {
    setStatus(NodeStatus::SUCCESS);
    std::unique_ptr<Feedback> fb_res = resultToFeedback(result);
    if (fb_res != nullptr)
      setOutput("feedback", *fb_res);
    return NodeStatus::SUCCESS;
  }

  /// Called when a service call failed. Can be overriden by the user.
  virtual NodeStatus onFailure(ActionNodeErrorCode failure)
  {
    return NodeStatus::FAILURE;
  }

  /// If you override this method, you MUST call this implementation invoking:
  ///
  ///    BaseClass::halt()
  ///
  virtual void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      action_client_->cancelGoal();
    }
  }

  void onResultCb(const actionlib::SimpleClientGoalState& state,
                  const WrappedResult& res)
  {
    switch (state.state_)
    {
      case actionlib::SimpleClientGoalState::SUCCEEDED:
      case actionlib::SimpleClientGoalState::PREEMPTED:
        setStatus(onResult(res));
        break;

      case actionlib::SimpleClientGoalState::RECALLED:
        setStatus(onFailure( ACTION_CANCELLED ));
        break;
      
      case actionlib::SimpleClientGoalState::ABORTED:
        setStatus(onFailure( ACTION_ABORTED ));
        break;
      
      case actionlib::SimpleClientGoalState::REJECTED:
        setStatus(onFailure( GOAL_REJECTED_BY_SERVER ));
        break;

      case actionlib::SimpleClientGoalState::LOST:
        setStatus(onFailure( INVALID_GOAL ));
        break;
      
      default:
        throw std::logic_error("Unexpected state in Action Result!");
        break;
    }
  }

  inline void onActiveCb(void){ setStatus(NodeStatus::RUNNING); }
  
  inline void onFeedbackCb(const FeedbackConstPtr& fb)
  {
    setOutput("feedback", *fb);
  }

protected:

  std::shared_ptr<ActionClient> action_client_;

  ros::NodeHandle& node_;

  BT::NodeStatus tick() override
  {
    // first step to be done only at the beginning of the Action
    if (status() == NodeStatus::IDLE) {
      BT::Result inRes;
      std::string server_name;
      if ( !(inRes = getInput<std::string>("server_name", server_name)))
          throw(BT::RuntimeError(
            "ROS Action Node missing required input [server_name]: ",
            inRes.error()));
      action_client_ = std::make_shared<ActionClient>
                        ( node_, server_name, true );
      unsigned msec = getInput<unsigned>("timeout").value();
      ros::Duration timeout(static_cast<double>(msec) * 1e-3);

      bool connected = action_client_->isServerConnected();
      if( !connected ){
        connected = action_client_->waitForServer(timeout);
        if( !connected )
          return onFailure(SERVER_UNREACHABLE);
      }
      
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(NodeStatus::RUNNING);

      Goal goal;
      bool valid_goal = sendGoal(goal);
      if( !valid_goal )
      {
        setStatus(NodeStatus::FAILURE);
        return NodeStatus::FAILURE;
      }
      action_client_->sendGoal(goal,
            boost::bind(&RosActionNode<ActionType>::onResultCb, this, ::_1, ::_2),
            boost::bind(&RosActionNode<ActionType>::onActiveCb, this),
            boost::bind(&RosActionNode<ActionType>::onFeedbackCb, this, ::_1));
    }

    /*
    // RUNNING
    actionlib::SimpleClientGoalState action_state = action_client_->getState();

    // Please refer to these states
    switch (action_state.state_)
    {
      case actionlib::SimpleClientGoalState::PENDING:
      case actionlib::SimpleClientGoalState::ACTIVE:
        break;

      case actionlib::SimpleClientGoalState::SUCCEEDED:
      case actionlib::SimpleClientGoalState::PREEMPTED:
      case actionlib::SimpleClientGoalState::RECALLED:
        setStatus(onResult(action_client_->getResult()));
        break;
      
      case actionlib::SimpleClientGoalState::ABORTED:
        setStatus(onFailure( ACTION_ABORTED ));
        break;
      
      case actionlib::SimpleClientGoalState::REJECTED:
        setStatus(onFailure( GOAL_REJECTED_BY_SERVER ));
        break;

      case actionlib::SimpleClientGoalState::LOST:
        setStatus(onFailure( GOAL_LOST ));
        break;
      
      default:
        throw std::logic_error("Unexpected state in RosActionNode::tick()");
        break;
    }
    */
    
    return status();
  }
};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosAction(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name,
                                      const NodeConfiguration& config)
  {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports =
              RosActionNode<typename DerivedT::ActionType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );
  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_ACTION_NODE_HPP_
