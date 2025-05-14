#include "sim_bt/pop_location_node.hpp"
#include <behaviortree_cpp/bt_factory.h>

namespace sim_bt {

PopLocation::PopLocation(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

BT::NodeStatus PopLocation::tick() {
  std::vector<geometry_msgs::msg::PoseStamped> locations;
  if (!getInput("locations_in", locations)) {
    throw BT::RuntimeError("PopLocation: missing input [locations_in]!");
  }

  if (!locations.empty()) {
    locations.erase(locations.begin());
    setOutput("locations_out", locations);
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace sim_bt

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<sim_bt::PopLocation>("PopLocation");
}