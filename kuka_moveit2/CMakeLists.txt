cmake_minimum_required(VERSION 3.5)
project(kuka_moveit2)

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
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)


set(DEPENDENCIES
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "moveit_ros_planning_interface"
  "Boost"
)

add_executable(run_moveit_cpp src/run_moveit.cpp)
ament_target_dependencies(run_moveit_cpp ${DEPENDENCIES})

add_executable(move_to_pose src/move_to_pose.cpp)
ament_target_dependencies(move_to_pose ${DEPENDENCIES})

install(TARGETS run_moveit_cpp
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS move_to_pose
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
