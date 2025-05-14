#include "sim_bt/check_locations_node.hpp"
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace sim_bt {

CheckLocations::CheckLocations(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

BT::NodeStatus CheckLocations::tick() {
  std::vector<geometry_msgs::msg::PoseStamped> locations;
  if (!getInput("locations", locations)) {
    throw BT::RuntimeError("CheckLocations: missing input [locations]!");
  }
  return locations.empty() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

} // namespace sim_bt

BT_REGISTER_NODES(factory) 
{
  factory.registerNodeType<sim_bt::CheckLocations>("CheckLocations");
}