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


# Install the python module for this package
ament_python_install_package(scripts/)


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


#install(TARGETS
#  dummy_joint_states
#  DESTINATION lib/${PROJECT_NAME}
#)

# Install python scripts
install(
  PROGRAMS
    scripts/server.py
    DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/client_lib.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/TCPconnector_withListener.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
