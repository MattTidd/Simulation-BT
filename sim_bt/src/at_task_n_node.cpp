#include <sim_bt/at_task_n_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace sim_bt
{
    // constructor:
    AtTaskN::AtTaskN(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    // get node info from blackboard:
    auto node = config.blackboard->get<rclcpp::Node*>("node");

    // declare & get parameters:
    node->declare_parameter("task_treshold", 0.5);
    node->get_parameter("task_threshold", threshold_);

    // set buffer & listener:
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus AtTaskN::tick()
{
    // get the message and handle missing case:
    std::vector<geometry_msgs::msg::PoseStamped> locations;
    if (!getInput("locations", locations)){
        throw BT::RuntimeError("AtTaskN: missing input [locations]!")
    }

    // check the first waypoint:
    const auto& target = locations.front()

    // 
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(rclcpp::get_logger("AtTaskN"), "%s", ex.what());
      return BT::NodeStatus::FAILURE;
    }

    double dx = tf.transform.translation.x - target.pose.position.x;
    double dy = tf.transform.translation.y - target.pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    return (dist < threshold_)
                ? BT::NodeStatus::SUCCESS
                : BT::NodeStatus::FAILURE;
}

}   // namespace sim_bt