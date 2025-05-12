#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "sim_bt/get_locations_node.hpp"
#include "sim_bt/at_task_n_node.hpp"
#include "sim_bt/go_to_n_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sim_bt");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<GetLocations>("GetLocations");
  factory.registerNodeType<AtTaskN>("AtTaskN");
  factory.registerNodeType<GoToN>("GoToN");
  // when I make the inspection node, also add it here!

  auto tree = factory.createTreeFromFile(
    node->declare_parameter("tree_file").get<std::string>(),
    node->get_node_logger().get_name()
  );

  // ROS2 timer to tick the tree at 10 Hz
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [&]() {
      tree.rootNode()->executeTick();
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
