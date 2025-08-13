cmake_minimum_required(VERSION 3.8)
project(bms_can_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(fake_can_pub src/fake_can_pub.cpp)
ament_target_dependencies(fake_can_pub rclcpp can_msgs)

add_executable(bms_state_node src/bms_state_node.cpp)
ament_target_dependencies(bms_state_node rclcpp can_msgs sensor_msgs std_msgs)

install(TARGETS
  fake_can_pub
  bms_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
