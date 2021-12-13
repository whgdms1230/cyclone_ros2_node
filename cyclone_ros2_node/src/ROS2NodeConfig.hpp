
#ifndef CYCLONE_ROS2_NODE__SRC__ROS2NODECONFIG_HPP
#define CYCLONE_ROS2_NODE__SRC__ROS2NODECONFIG_HPP

#include <string>

namespace cyclone_bridge
{
namespace ros2
{

struct ROS2NodeConfig
{
  std::string ros1_to_ros2_topic = "/ros1_to_ros2_topic";
  std::string ros2_to_ros1_topic = "/ros2_to_ros1_topic";

  int dds_domain = 100;

  std::string dds_ros1_to_ros2_topic = "ros1_to_ros2";
  std::string dds_ros2_to_ros1_topic = "ros2_to_ros1";

  ROS2Config get_ros2_config() const;

  static ROS2NodeConfig make();

};

} // namespace ros2
} // namespace cyclone_bridge

#endif // CYCLONE_ROS2_NODE__SRC__ROS2NODECONFIG_HPP
