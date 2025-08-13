cmake_minimum_required(VERSION 3.8)
project(bms_can_bridge_cpp)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 查找Qt包 - 优先使用Qt5
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
if(NOT Qt5_FOUND)
  find_package(Qt6 COMPONENTS Core Widgets REQUIRED)
endif()

# 设置Qt自动处理
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 包含目录
include_directories(include)

# 创建BMS显示系统可执行文件
add_executable(bms_display_system
  src/bms_display_main.cpp
  src/bms_display_ui.cpp
)

# 链接Qt库
if(Qt5_FOUND)
  target_link_libraries(bms_display_system
    Qt5::Core
    Qt5::Widgets
  )
  message(STATUS "Using Qt5")
else()
  target_link_libraries(bms_display_system
    Qt6::Core
    Qt6::Widgets
  )
  message(STATUS "Using Qt6")
endif()

# 链接ROS2库
target_link_libraries(bms_display_system
  ${rclcpp_LIBRARIES}
)

# 包含ROS2头文件目录
target_include_directories(bms_display_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 创建原有的可执行文件
add_executable(fake_can_pub src/fake_can_pub.cpp)
ament_target_dependencies(fake_can_pub rclcpp can_msgs)

add_executable(bms_state_node src/bms_state_node.cpp)
ament_target_dependencies(bms_state_node rclcpp sensor_msgs std_msgs)

# 安装目标
install(TARGETS
  bms_display_system
  fake_can_pub
  bms_state_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装脚本
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  can_msgs
  sensor_msgs
  std_msgs
)

# 导出目标
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGETS)
ament_export_libraries(${PROJECT_NAME})

# 导出包含目录
ament_export_include_directories(include)

# 导出库目录
ament_export_library_directories(lib)

# 导出依赖
ament_package()
