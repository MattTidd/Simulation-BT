#ifndef SIM_BT__GO_TO_N_NODE_HPP_
#define SIM_BT__GO_TO_N_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace sim_bt
{

class GoToN : public BT::SyncActionNode
{
public:
  GoToN(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("locations") };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Node* node_;
  rclcpp::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  std::shared_future<typename nav2_msgs::action::NavigateToPose::Result::SharedPtr> result_future_;
  bool goal_sent_;
};

}  // namespace sim_bt

#endif  // SIM_BT__GO_TO_N_NODE_HPP_
