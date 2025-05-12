#ifndef SIM_BT__AT_TASK_N_NODE_HPP_
#define SIM_BT__AT_TASK_N_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace sim_bt
{
class AtTaskN : public BT::SyncActionNode
{
public:
    // constructor:
    AtTaskN(const std::string& name, const BT::NodeConfiguration& config);

    // list the ports:
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("locations")};
    }

    // tick method:
    BT::NodeStatus tick() override;

private:
    double treshold_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}   // namespace sim_bt

#endif  // SIM_BT__AT_TASK_N_NODE_HPP_