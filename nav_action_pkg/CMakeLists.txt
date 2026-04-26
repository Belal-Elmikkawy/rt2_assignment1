cmake_minimum_required(VERSION 3.8)
project(nav_action_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Navigate.action"
  "msg/UserCommand.msg"
  DEPENDENCIES geometry_msgs
)

# Get typesupport target for linking custom messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# ==========================================================
# 1. BUILD THE ACTION SERVER COMPONENT
# ==========================================================
add_library(action_server_component SHARED src/action_server_component.cpp)
target_include_directories(action_server_component PRIVATE include)
target_compile_features(action_server_component PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(action_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)
target_link_libraries(action_server_component "${cpp_typesupport_target}")

rclcpp_components_register_node(action_server_component
  PLUGIN "nav_action_pkg::ActionServerComponent"
  EXECUTABLE action_server_component_node
)

# ==========================================================
# 2. BUILD THE ACTION CLIENT COMPONENT
# ==========================================================
add_library(action_client_component SHARED src/action_client_component.cpp)
target_include_directories(action_client_component PRIVATE include)
target_compile_features(action_client_component PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(action_client_component
  rclcpp
  rclcpp_action
  rclcpp_components
)
target_link_libraries(action_client_component "${cpp_typesupport_target}")

rclcpp_components_register_node(action_client_component
  PLUGIN "nav_action_pkg::ActionClientComponent"
  EXECUTABLE action_client_component_node
)

# ==========================================================
# 3. INSTALL RULES
# ==========================================================
install(TARGETS
  action_server_component
  action_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()