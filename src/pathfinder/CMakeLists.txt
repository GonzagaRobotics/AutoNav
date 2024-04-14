cmake_minimum_required(VERSION 3.8)
project(pathfinder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(auto_nav_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  ${auto_nav_interfaces_INCLUDE_DIRS}
)

add_executable(pathfinder
  src/deps/fsa.h
  src/deps/astar.h
  src/deps/stb_image.h
  src/deps/stb_image_write.h
  src/Site.h
  src/Site.cpp
  src/SiteLoader.h
  src/SiteLoader.cpp
  src/Search.h
  src/Search.cpp
  src/debug/DebugKML.h
  src/debug/DebugKML.cpp
  src/Main.cpp)
ament_target_dependencies(pathfinder rclcpp std_msgs auto_nav_interfaces)

install(TARGETS pathfinder DESTINATION lib/${PROJECT_NAME})

ament_package()
