cmake_minimum_required(VERSION 3.8)
project(auto_nav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GeoLoc.msg"
  "msg/Plan.msg"
  "msg/State.msg"
  "msg/Target.msg"
  "msg/Instruction.msg"
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()