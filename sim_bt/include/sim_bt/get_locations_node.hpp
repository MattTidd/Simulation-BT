#ifndef SIM_BT__GET_LOCATIONS_NODE_HPP_
#define SIM_BT__GET_LOCATIONS_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

namespace sim_bt
{

class GetLocations : public BT::SyncActionNode
{
public:
    // constructor:
    GetLocations(const std::string& name, const BT::NodeConfiguration& config);

    // list the ports:
    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("locations") };
    }

    // tick method:
    BT::NodeStatus tick() override;

private:
    std::vector<geometry_msgs::msg::PoseStamped> locations_;
};
 
}   // namespace sim_bt

#endif // SIM_BT__GET_LOCATIONS_NODE_HPP_
