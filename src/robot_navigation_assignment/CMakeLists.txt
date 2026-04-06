cmake_minimum_required(VERSION 3.8)
project(robot_navigation_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Action definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
)

ament_export_dependencies(rosidl_default_runtime)

# Libraries (components)
add_library(navigation_server SHARED src/navigation_server.cpp)
ament_target_dependencies(navigation_server
  rclcpp
  rclcpp_components
  rclcpp_action
)

add_library(navigation_client_ui SHARED src/navigation_client_ui.cpp)
ament_target_dependencies(navigation_client_ui
  rclcpp
  rclcpp_components
  rclcpp_action
)

# Register components
rclcpp_components_register_nodes(navigation_server
  "robot_navigation_assignment::NavigationServer")

rclcpp_components_register_nodes(navigation_client_ui
  "robot_navigation_assignment::NavigationClientUI")

# Install
install(TARGETS
  navigation_server
  navigation_client_ui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()