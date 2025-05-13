#include <sim_bt/get_locations_node.hpp>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

namespace sim_bt
{
    // constructor:
    GetLocations::GetLocations(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    // get node info from blackboard:
    auto node = config.blackboard->get<rclcpp::Node*>("node");

    // declare & get a parameter containing the list of poses, flattened as x,y, yaw:
    node->declare_parameter<std::vector<double>>("task_locations", {});
    std::vector<double> flat;
    node->get_parameter("task_locations", flat);

    // convert [x0, y0, yaw0, x1, y1, yaw1, ...] into PoseStamped list:
    for (size_t i = 0; i+2 < flat.size(); i +=3) {
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = flat[i];
        p.pose.position.y = flat[i + 1];
        tf2::Quaternion q;
        q.setRPY(0,0, flat[i+2]);
        p.pose.orientation = tf2::toMsg(q);
        locations_.push_back(p);
    }  
}

BT::NodeStatus GetLocations::tick()
{
    setOutput("locations", locations_);
    std::cout << "Got Locations!" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

}   // namespace sim_bt

// register with the factory:
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<sim_bt::GetLocations>("GetLocations");
}
