cmake_minimum_required(VERSION 3.5.0)

project(cyclone_ros2_msgs LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
  find_package(builtin_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(cyclone_ros2_msgs
    "msg/IntNumber.msg"
    "msg/StrString.msg"
    "msg/Msg.msg"
    DEPENDENCIES std_msgs
  )

  ament_package()

else()
  message("ament_cmake not found so skipping this ROS2 package")
endif()
