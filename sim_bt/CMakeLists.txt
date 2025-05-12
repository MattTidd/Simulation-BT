cmake_minimum_required(VERSION 3.8)
project(sim_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(action_msgs REQUIRED)

# include the headers:
include_directories(
  include
)

# make the BT node executable:
add_executable(bt_node
  src/main.cpp 
  src/get_locations_node.cpp 
  src/at_task_n_node.cpp 
  src/go_to_n_node.cpp 
)

ament_target_dependencies(bt_node
  rclcpp
  behaviortree_cpp
  nav2_msgs
  geometry_msgs
  tf2_ros
  action_msgs
)

# install everything:
install(TARGETS bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
