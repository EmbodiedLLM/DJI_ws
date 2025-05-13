cmake_minimum_required(VERSION 3.5)
project(dji_receiver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  include/dji_receiver
)

add_executable(dji_receiver_node
  src/dji_receiver_node.cpp
  src/CRC8_CRC16.c
)

target_link_libraries(dji_receiver_node
)

target_include_directories(dji_receiver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/dji_receiver>
)

ament_target_dependencies(dji_receiver_node
  rclcpp
  nav_msgs
  geometry_msgs
  serial
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS dji_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# 添加 launch 和 config 目录的安装规则
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package() 