#ifndef SIM_BT__CHECK_LOCATIONS_NODE_HPP_
#define SIM_BT__CHECK_LOCATIONS_NODE_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

namespace sim_bt {

class CheckLocations : public BT::ConditionNode {
public:
  CheckLocations(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("locations") };
  }

  BT::NodeStatus tick() override;
};

} // namespace sim_bt

#endif // SIM_BT__CHECK_LOCATIONS_NODE_HPP_