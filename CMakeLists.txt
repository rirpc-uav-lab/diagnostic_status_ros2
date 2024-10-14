cmake_minimum_required(VERSION 3.8)
project(diagnostic_status_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)

set(COLCON_VARS ament_cmake diagnostic_msgs yaml-cpp rclcpp) # roslib

install(DIRECTORY
 config
 DESTINATION share/${PROJECT_NAME}
)

include_directories(include)

add_executable(diagnostic_module src/diagnostic_module.cpp)
target_link_libraries(diagnostic_module yaml-cpp)
ament_target_dependencies(diagnostic_module ${COLCON_VARS})

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

install(TARGETS
  diagnostic_module
  DESTINATION lib/${PROJECT_NAME})

ament_package()
