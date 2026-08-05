cmake_minimum_required(VERSION 3.8)
project(marsdog_action_executor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# NOTE: ExecuteBehavior.action has been migrated to marsdog_interfaces.
# The ros_node will prefer marsdog_interfaces.action.ExecuteBehavior when
# available, but we still generate the local copy as a fallback during
# the transition period.  Remove this block once marsdog_interfaces is
# available in all workspaces.
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "action/ExecuteBehavior.action"
)

# Install our Python modules
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/marsdog_action_executor/
  DESTINATION ${PYTHON_INSTALL_DIR}/marsdog_action_executor
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install launcher scripts
install(
  PROGRAMS
    scripts/action_executor_node
    scripts/emotion_display
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
