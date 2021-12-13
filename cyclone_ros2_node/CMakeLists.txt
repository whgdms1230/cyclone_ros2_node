cmake_minimum_required(VERSION 3.5.0)

project(cyclone_ros2_node LANGUAGES C CXX)

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
  find_package(cyclone_bridge REQUIRED)
  find_package(cyclone_ros2_msgs)

  add_executable(cyclone_ros2_node
    src/main.cpp
    src/ROS2Node.cpp
    src/ROS2NodeConfig.cpp
  )
  target_link_libraries(cyclone_ros2_node
    ${cyclone_bridge_LIBRARIES}
  )
  target_include_directories(cyclone_ros2_node
    PRIVATE
      ${cyclone_bridge_INCLUDE_DIRS}
  )
  ament_target_dependencies(cyclone_ros2_node
    rclcpp
    cyclone_ros2_msgs
  )
  
  install(
    TARGETS cyclone_ros2_node
    RUNTIME DESTINATION lib/cyclone_ros2_node
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
  
  ament_package()

else()
  message("ament_cmake not found so skipping this ROS2 package")
endif()
