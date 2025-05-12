#include "sim_bt/go_to_n_node.hpp"
#include <behaviortree_cpp/bt_factory.h>

namespace sim_bt
{

GoToN::GoToN(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config), goal_sent_(false)
{
  node_ = config.blackboard->get<rclcpp::Node*>("node");
  action_client_ = rclcpp::create_client<nav2_msgs::action::NavigateToPose>(
    node_, "navigate_to_pose");
}

BT::NodeStatus GoToN::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> locations;
  if (!getInput("locations", locations)) {
    throw BT::RuntimeError("GoToN: missing input [locations]");
  }
  const auto& target = locations.front();

  if (!goal_sent_) {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available");
      return BT::NodeStatus::FAILURE;
    }
    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = target;
    auto goal_handle_future = action_client_->async_send_goal(goal_msg);
    // chain into result
    result_future_ = goal_handle_future.then(
      [](auto fut) { return fut.get()->async_result(); });
    goal_sent_ = true;
    return BT::NodeStatus::RUNNING;
  }

  // check result
  if (result_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    auto result = result_future_.get();
    goal_sent_ = false;
    return (result->status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void GoToN::halt()
{
  // Optionally cancel the goal here
  goal_sent_ = false;
}

}  // namespace sim_bt

// Register with factory
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sim_bt::GoToN>("GoToN");
}
