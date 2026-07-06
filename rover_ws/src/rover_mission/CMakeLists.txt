cmake_minimum_required(VERSION 3.8)
project(rover_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_msgs
  geometry_msgs
  std_msgs
)

include_directories(include)

add_executable(mission_server
  src/mission_server.cpp
  src/bt_nodes.cpp
)
ament_target_dependencies(mission_server ${dependencies})

install(TARGETS mission_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # This project does not enforce copyright headers or C++ formatting style
  # (uncrustify/cpplint); skip those linters so CI reflects functional health.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
