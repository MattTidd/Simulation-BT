#include <sim_bt/go_to_n_node.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp_action/client_goal_handle.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace sim_bt
{
  // constructor:
  GoToN::GoToN(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config),
  goal_sent_(false),
  node_(nullptr)
{
  node_ = config.blackboard->get<rclcpp::Node*>("node");
  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node_, "navigate_to_pose");
}

// tick method:
BT::NodeStatus GoToN::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> locations;
  // if no input:
  if (!getInput("locations", locations)){
    throw BT::RuntimeError("GoToN: missing required input [locations]!");
  }

  // if inputs empty:
  if (locations.empty()){
    RCLCPP_ERROR(node_->get_logger(), "No locations provided!");
    return BT::NodeStatus::FAILURE;
  }

  // get the first message:
  const auto& target = locations.front();

  // goal handling:
  if(!goal_sent_) {
    // initialize the action client if not ready:
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))){
      RCLCPP_ERROR(node_->get_logger(), "Action server not available");
      return BT::NodeStatus::FAILURE;
    }

    // create and send a goal:
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = target;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = 
    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(node_->get_logger(), "Navigation failed with result code: %d", static_cast<int>(result.code));
      }
    };

    auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);

    // goal send failure:
    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to send goal!");
      return BT::NodeStatus::FAILURE;
    }

    // server rejection case:
    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_){
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      return BT::NodeStatus::FAILURE;
    }

    goal_sent_ = true;
    return BT::NodeStatus::RUNNING;
  }

  // check if goal is still executing:
  auto status = goal_handle_->get_status();
  if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        return BT::NodeStatus::RUNNING;
      }
  
  // get final result:
  auto future_result = action_client_->async_get_result(goal_handle_);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future_result)
      != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get result!");
        goal_sent_= false;
        return BT::NodeStatus::FAILURE;
      }
  auto result = future_result.get();
  goal_sent_ = false;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

} // namespace sim_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sim_bt::GoToN>("GoToN");
}

