cmake_minimum_required(VERSION 3.8)
project(auto_nav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GeoPoint.msg"
  "msg/Plan.msg"
  "msg/State.msg"
  "msg/Target.msg"
  "msg/TargetType.msg"
  "msg/Instruction.msg"
)

ament_package()