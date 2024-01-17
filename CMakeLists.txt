cmake_minimum_required(VERSION 3.5)
project(tutorial_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROS2_WS_ROOT $ENV{ROS_WS})
set(ALICE3_PARAMETERS_PATH "${ROS2_WS_ROOT}/src/alice3_parameters")
add_definitions(-DALICE3_PARAMETERS_PATH="${ALICE3_PARAMETERS_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(pub_node src/pub_node.cpp)
ament_target_dependencies(pub_node rclcpp std_msgs)

add_executable(sub_node src/sub_node.cpp)
ament_target_dependencies(sub_node rclcpp std_msgs)

add_executable(service_server_node src/service_server_node.cpp)
ament_target_dependencies(service_server_node rclcpp tutorial_interfaces)

add_executable(service_client_node src/service_client_node.cpp)
ament_target_dependencies(service_client_node rclcpp tutorial_interfaces)

add_executable(action_server_node src/action_server_node.cpp)
ament_target_dependencies(action_server_node tutorial_interfaces rclcpp rclcpp_action)

add_executable(action_client_node src/action_client_node.cpp)
ament_target_dependencies(action_client_node tutorial_interfaces rclcpp rclcpp_action)

add_executable(get_path src/get_path.cpp)
ament_target_dependencies(get_path rclcpp std_msgs)

install(TARGETS
  pub_node
  sub_node
  service_server_node
  service_client_node
  action_server_node
  action_client_node
  get_path
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
