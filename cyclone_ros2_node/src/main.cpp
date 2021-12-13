
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "ROS2Node.hpp"


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Greetings from cyclone_ros2_node" << std::endl;

  cyclone_bridge::ros2::ROS2NodeConfig ros2_node_config =
      cyclone_bridge::ros2::ROS2NodeConfig::make();

  auto ros2_node = cyclone_bridge::ros2::ROS2Node::make(ros2_node_config);
  if (!ros2_node)
    return 1;

  rclcpp::executors::MultiThreadedExecutor executor {
      rclcpp::ExecutorOptions(), 2};
  executor.add_node(ros2_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
