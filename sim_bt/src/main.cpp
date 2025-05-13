#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "sim_bt/get_locations_node.hpp"
#include "sim_bt/at_task_n_node.hpp"
#include "sim_bt/go_to_n_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sim_bt");

  try {
    // initialize behavior tree factory:
    BT::BehaviorTreeFactory factory;
    
    // register custom nodes
    factory.registerNodeType<sim_bt::GetLocations>("GetLocations");
    factory.registerNodeType<sim_bt::AtTaskN>("AtTaskN"); 
    factory.registerNodeType<sim_bt::GoToN>("GoToN");
    // add the inspection node when it is made!

    // get package path:
    const std::string package_share_dir = ament_index_cpp::get_package_share_directory("sim_bt");

    // load behavior tree from XML file using absolute path:
    const std::string tree_filename = node->declare_parameter<std::string>(
      "tree_file",      // try this
      package_share_dir + "/trees/my_tree.xml"  // fallback to this
    );

    // get the task_count parameter:
    int task_count = node->declare_parameter<int>("task_count", 1);

    // debug:
    RCLCPP_INFO(node->get_logger(), "Loading tree from: %s", tree_filename.c_str());

    // create the tree:
    auto blackboard = BT::Blackboard::create();    // create a blackboard
    blackboard->set("node", node.get()); // send the node to the blackboard
    blackboard->set("task_count", task_count);  // send the number of tasks to the blackboard
    RCLCPP_INFO(node->get_logger(), "Passed task count: %d", task_count);
    auto tree = factory.createTreeFromFile(tree_filename, blackboard);  // create tree

    // configure tree tick timer:
    const auto tick_period = std::chrono::milliseconds(
      node->declare_parameter<int>("tick_rate_ms", 100)
    );

    auto timer = node->create_wall_timer(tick_period, [&]() {
      try {
        const BT::NodeStatus status = tree.tickOnce();
        if(status == BT::NodeStatus::RUNNING) {
          return;
        }
        
        // shutdown when tree completes:
        RCLCPP_INFO(node->get_logger(), "Behavior Tree completed with status: %d", static_cast<int>(status));
        rclcpp::shutdown();
        
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error executing tree: %s", e.what());
        rclcpp::shutdown();
      }
    });

    RCLCPP_INFO(node->get_logger(), "Behavior Tree controller started!");
    rclcpp::spin(node);

  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize: %s", e.what());
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}