
#ifndef CYCLONE_ROS2_NODE__SRC__ROS2NODE_HPP
#define CYCLONE_ROS2_NODE__SRC__ROS2NODE_HPP

#include <mutex>
#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>

#include <cyclone_ros2_msgs/msg/int_number.hpp>

#include <cyclone_bridge/ROS2Bridge.hpp>

#include <cyclone_bridge/messages/IntNumber.hpp>

#include "ROS2NodeConfig.hpp"

namespace cyclone_bridge
{
namespace ros2
{

class ROS2Node : public rclcpp::Node
{
public:

  using SharedPtr = std::shared_ptr<ROS2Node>;
  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  static SharedPtr make(
      const ROS2NodeConfig& config,
      const rclcpp::NodeOptions& options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true));

  ~ROS2Node();

  struct Fields
  {
    ROS2Bridge::SharedPtr ros2_bridge;
  };

private:

  rclcpp::Publisher<cyclone_ros2_msgs::msg::IntNumber>::SharedPtr
      ros1_to_ros2_num_pub;

  rclcpp::CallbackGroup::SharedPtr read_callback_group;

  rclcpp::TimerBase::SharedPtr read_timer;

  void read();

  void send();

  ROS2NodeConfig ros2_node_config;

  Fields fields;

  ROS2Node(
      const ROS2NodeConfig& config, const rclcpp::NodeOptions& options);

  void start(Fields fields);

  uint32_t return_number;

};

} // namespace ros2
} // namespace cyclone_bridge

#endif // CYCLONE_ROS2_NODE__SRC__ROS2NODE_HPP
