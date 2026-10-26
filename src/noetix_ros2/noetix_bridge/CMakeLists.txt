cmake_minimum_required(VERSION 3.8)
project(noetix_bridge)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(NOETIX_YAML_TARGET yaml-cpp::yaml-cpp)
else()
  set(NOETIX_YAML_TARGET yaml-cpp)
endif()
find_package(noetix_msgs REQUIRED)
find_package(noetix_sdk2 CONFIG REQUIRED)

# Share configuration loading without linking utility nodes to the hardware SDK.
add_library(noetix_bridge_resources STATIC
  src/robot_resource.cpp
)
target_compile_features(noetix_bridge_resources PUBLIC cxx_std_17)
target_include_directories(noetix_bridge_resources PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(noetix_bridge_resources
  ${NOETIX_YAML_TARGET}
)
ament_target_dependencies(noetix_bridge_resources
  ament_index_cpp
)

add_executable(noetix_bridge_node
  src/noetix_bridge_node.cpp
  src/io_loop.cpp
)
target_compile_features(noetix_bridge_node PUBLIC cxx_std_17)
target_link_libraries(noetix_bridge_node
  noetix_bridge_resources
  noetix::noetix_sdk2
  Threads::Threads
)
ament_target_dependencies(noetix_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  noetix_msgs
)

# Optional upper-layer utility.  It publishes ordinary LowCmd packets and has
# no privileged path into the bridge.
add_executable(noetix_pose_mode_node src/pose_mode_node.cpp)
target_compile_features(noetix_pose_mode_node PUBLIC cxx_std_17)
target_link_libraries(noetix_pose_mode_node noetix_bridge_resources)
ament_target_dependencies(noetix_pose_mode_node
  rclcpp
  std_msgs
  noetix_msgs
)

install(TARGETS
  noetix_bridge_node
  noetix_pose_mode_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
