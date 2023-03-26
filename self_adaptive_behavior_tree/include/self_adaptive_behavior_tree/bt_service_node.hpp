#ifndef BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_
#define BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_

#include <memory>
#include <string>

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace BT {

struct RosServiceNodeParams {
  std::shared_ptr<rclcpp::Node> nh;
  std::string service_name;
  std::chrono::milliseconds server_timeout;

  RosServiceNodeParams(const std::shared_ptr<rclcpp::Node>& node,
                       const std::string& service_name = "", const unsigned int& server_timeout = 0)
      : nh(node), service_name(service_name), server_timeout(server_timeout){};
};

enum class ServiceNodeErrorCode {
  SERVER_UNREACHABLE,
  PROCESS_REQUEST_TIMEOUT,
  REQUEST_REJECTED_BY_SERVER,
  INVALID_REQUST
};

/**
 * @brief Abstract class representing use to call a ROS2 Service (client).
 *
 * It will try to be non-blocking for the entire duration of the call.
 * The derived class whould reimplement the virtual methods as described below.
 */
template <class ServiceT>
class RosServiceNode : public SyncActionNode {
 public:
  // Type definitions
  using ServiceType = ServiceT;
  using ServiceClient = typename rclcpp::Client<ServiceT>;
  using Request = typename ServiceT::Request;
  using RequestHandle = typename rclcpp::Client<ServiceT>::FutureAndRequestId;
  using Response = typename ServiceT::Response;
  using Params = RosServiceNodeParams;
  using Error = ServiceNodeErrorCode;

  /** You are not supposed to instantiate this class directly, the factory will do it.
   * To register this class into the factory, use:
   *
   *    RegisterRosService<DerivedClasss>(factory, params)
   *
   * Note that if the external_service_client is not set, the constructor will build its own.
   * */
  explicit RosServiceNode(const std::string& instance_name, const NodeConfiguration& conf,
                          const RosServiceNodeParams& params,
                          typename std::shared_ptr<ServiceClient> external_service_client = {});

  virtual ~RosServiceNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosService<DeriveClass>()
  static PortsList providedPorts() {
    return {InputPort<std::string>("service_name", "name of the ROS service"),
            InputPort<unsigned>("timeout", 100, "timeout to connect to server (milliseconds)")};
  }

  NodeStatus tick() override final;

  /** sendRequest is a callback invoked to return the request message (ServiceT::Request).
   * If conditions are not met, it should return "false" and the Action
   * will return FAILURE.
   */
  virtual bool sendRequest(Request& request) = 0;

  /** Callback invoked when the result is received from the server.
   * It is up to the user to define if the action returns SUCCESS or FAILURE.
   */
  virtual NodeStatus onResponse(const Response& rep) {
    (void)rep;
    return NodeStatus::SUCCESS;
  };

  /** Callback invoked when something goes wrong.
   * It must return either SUCCESS or FAILURE.
   */
  virtual NodeStatus onFailure(const Error& error) {
    (void)error;
    return NodeStatus::FAILURE;
  }

 protected:
  std::shared_ptr<rclcpp::Node> node_;
  const std::string service_name_;
  const std::chrono::milliseconds server_timeout_;

 private:
  NodeStatus checkStatus(const NodeStatus& status) {
    if (status != NodeStatus::SUCCESS && status != NodeStatus::FAILURE) {
      throw std::logic_error("RosActionNode: the callback must return either SUCCESS of FAILURE");
    }
    return status;
  };

  typename std::shared_ptr<ServiceClient> service_client_;

  std::unique_ptr<RequestHandle> request_handle_;

  rclcpp::Time time_request_sent_;
  // NodeStatus on_feedback_state_change_;
  bool has_response_;
  // WrappedResult result_;
};

/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT>
static void RegisterRosService(
    BehaviorTreeFactory& factory, const std::string& registration_ID,
    const RosServiceNodeParams& params,
    std::shared_ptr<typename DerivedT::ServiceClient> external_client = {}) {
  NodeBuilder builder = [=](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(name, config, params, external_client);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = DerivedT::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class T>
inline RosServiceNode<T>::RosServiceNode(
    const std::string& instance_name, const NodeConfiguration& conf,
    const RosServiceNodeParams& params,
    typename std::shared_ptr<ServiceClient> external_service_client)
    : SyncActionNode(instance_name, conf),
      node_(params.nh),
      service_name_(params.service_name),
      server_timeout_(params.server_timeout) {
  if (external_service_client) {
    service_client_ = external_service_client;
  } else {
    service_client_ = nullptr;
  }
};

template <class T>
inline NodeStatus RosServiceNode<T>::tick() {
  // first step to be done only at the beginning of the Action
  if (status() == NodeStatus::IDLE) {
    if (service_client_ == nullptr) {
      std::string service_name = "";
      Result inRes;
      if (!(inRes = getInput<std::string>("service_name", service_name)))
        service_name = service_name_;
      if (service_name.length() == 0) service_name = service_name_;
      unsigned server_timeout = 0;
      if (!(inRes = getInput<unsigned>("timeout", server_timeout)))
        server_timeout = server_timeout_.count();
      if (server_timeout == 0) server_timeout = server_timeout_.count();
      service_client_ = node_->create_client<T>(service_name);
      bool found = service_client_->wait_for_service(std::chrono::milliseconds(server_timeout));
      if (!found) {
        RCLCPP_ERROR(node_->get_logger(), "Service [%s] is not reachable.", service_name.c_str());
        return checkStatus(onFailure(Error::SERVER_UNREACHABLE));
      }
    }

    request_handle_ = {};

    std::unique_ptr<Request> request(new Request);

    if (!sendRequest(*request)) {
      return checkStatus(onFailure(Error::INVALID_REQUST));
    }

    request_handle_ =
        std::make_unique<RequestHandle>(service_client_->async_send_request(std::move(request)));

    if (rclcpp::spin_until_future_complete(node_, *request_handle_, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", service_name_.c_str());
      return checkStatus(onFailure(Error::PROCESS_REQUEST_TIMEOUT));
    } else {
      if (request_handle_ == nullptr)
        return checkStatus(onFailure(Error::REQUEST_REJECTED_BY_SERVER));
      std::shared_ptr<Response> resp = request_handle_->get();
      if (resp == nullptr) return checkStatus(onFailure(Error::REQUEST_REJECTED_BY_SERVER));
      return checkStatus(onResponse(*resp));
    }
  }
  return NodeStatus::SUCCESS;
};

}  // namespace BT

#endif  // BEHAVIOR_TREE_ROS2__BT_SERVICE_NODE_HPP_
