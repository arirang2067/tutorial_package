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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(pub_node src/pub_node.cpp)
ament_target_dependencies(pub_node rclcpp std_msgs)

add_executable(sub_node src/sub_node.cpp)
ament_target_dependencies(sub_node rclcpp std_msgs)

add_executable(server_node src/server_node.cpp)
ament_target_dependencies(server_node rclcpp tutorial_interfaces)

add_executable(client_node src/client_node.cpp)
ament_target_dependencies(client_node rclcpp tutorial_interfaces)

install(TARGETS
  pub_node
  sub_node
  server_node
  client_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
