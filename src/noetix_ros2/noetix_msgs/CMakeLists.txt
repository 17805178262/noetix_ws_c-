cmake_minimum_required(VERSION 3.8)
project(noetix_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/MotorCmd.msg"
  "msg/ImuState.msg"
  "msg/RobotStatus.msg"
  "msg/LowState.msg"
  "msg/LowCmd.msg"
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
