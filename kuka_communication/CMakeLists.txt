cmake_minimum_required(VERSION 3.5)
project(kuka_communication)

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



################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
  "msg/KmpStatusdata.msg"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)


#add_executable(dummy_joint_states dummy_joint_states.cpp)
#ament_target_dependencies(dummy_joint_states "rclcpp" "sensor_msgs")


# Install C++ headers
#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/${PROJECT_NAME}
#  FILES_MATCHING PATTERN "*.hpp"
#)

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


# Install python scripts
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    script/server_UDP.py
    script/dummy_data.py
    script/view_frames.py
    script/buffer.py
    script/server_TCP.py
    script/test.py
    script/tcpSocket.py
    script/udpSocket.py
    nodes/kmp_commands_node.py
    nodes/kmp_laserscan_node.py
    nodes/kmp_odometry_node.py
    nodes/kmp_statusdata_node.py
    nodes/test2.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
