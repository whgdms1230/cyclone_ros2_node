
#include <cstdio>

#include <cyclone_bridge/ROS2Config.hpp>

#include "ROS2NodeConfig.hpp"

namespace cyclone_bridge
{
namespace ros2
{

ROS2Config ROS2NodeConfig::get_ros2_config() const
{
  ROS2Config ros2_config;
  ros2_config.dds_domain = dds_domain;
  ros2_config.dds_ros1_to_ros2_topic = dds_ros1_to_ros2_topic;
  ros2_config.dds_ros2_to_ros1_topic = dds_ros2_to_ros1_topic;
  return ros2_config;
}

ROS2NodeConfig ROS2NodeConfig::make()
{
  ROS2NodeConfig config;
  return config;
}

} // namespace ros2
} // namespace cyclone_bridge
