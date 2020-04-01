cmake_minimum_required(VERSION 3.5)
project(kuka_manipulator)

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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install the python module for this package
ament_python_install_package(script/)
ament_python_install_package(nodes/)



set(action_files
  "action/OpenGripper.action"
  "action/CloseGripper.action"
  "action/ObjectSearch.action"
  "action/PlanToFrame.action"
  "action/MoveManipulator.action"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  DIRECTORY launch action
  DESTINATION share/${PROJECT_NAME}
)

# Install python scripts
install(
  PROGRAMS
    script/errorcodes.py
    script/gripper_msgs.py
    nodes/manipulator_node.py
    nodes/gripper/gripper_node.py
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()
