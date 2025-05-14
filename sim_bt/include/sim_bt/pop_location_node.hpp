#ifndef SIM_BT__POP_LOCATION_NODE_HPP_
#define SIM_BT__POP_LOCATION_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

namespace sim_bt {

class PopLocation : public BT::SyncActionNode {
public:
  PopLocation(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("locations_in"),
             BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("locations_out") };
  }

  BT::NodeStatus tick() override;
};

} // namespace sim_bt

#endif // SIM_BT__POP_LOCATION_NODE_HPP_