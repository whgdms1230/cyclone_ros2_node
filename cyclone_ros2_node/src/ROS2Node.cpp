
#include <chrono>

#include <cyclone_bridge/ROS2Bridge.hpp>
#include <cyclone_bridge/ROS2Config.hpp>

#include "ROS2Node.hpp"

namespace cyclone_bridge
{
namespace ros2
{

ROS2Node::SharedPtr ROS2Node::make(
    const ROS2NodeConfig& _config, const rclcpp::NodeOptions& _node_options)
{
  // Starting the free fleet server node
  SharedPtr ros2_node(new ROS2Node(_config, _node_options));

  ROS2Config ros2_config =
      ros2_node->ros2_node_config.get_ros2_config();
  ROS2Bridge::SharedPtr ros2_bridge = ROS2Bridge::make(ros2_config);
  if (!ros2_bridge)
    return nullptr;

  ros2_node->start(Fields{
    std::move(ros2_bridge)
  });

  return ros2_node;
}

ROS2Node::~ROS2Node()
{}

ROS2Node::ROS2Node(
    const ROS2NodeConfig& _config,
    const rclcpp::NodeOptions& _node_options) :
  Node("cyclone_ros2_node", _node_options),
  ros2_node_config(_config)
{}

void ROS2Node::start(Fields _fields)
{
  fields = std::move(_fields);

  read_callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  read_timer = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&ROS2Node::read, this),
      read_callback_group);

  ros1_to_ros2_num_pub =
      create_publisher<cyclone_ros2_msgs::msg::IntNumber>(
          ros2_node_config.ros1_to_ros2_topic, 10);
}

void ROS2Node::read()
{
  messages::IntNumber ros1_to_ros2_num;
  if (fields.ros2_bridge->read(ros1_to_ros2_num))
  {
    cyclone_ros2_msgs::msg::IntNumber new_num;
    new_num.int_num = ros1_to_ros2_num.int_num;
    
    return_number = new_num.int_num;
    
    ros1_to_ros2_num_pub->publish(new_num);

    send();
  }
}

void ROS2Node::send()
{
  std::cout << "send" << std::endl;
  messages::IntNumber ros2_to_ros1_num;
  ros2_to_ros1_num.int_num = return_number;

  fields.ros2_bridge->send(ros2_to_ros1_num);
}


} // namespace ros2
} // namespace free_fleet
