cmake_minimum_required(VERSION 3.5)
project(kuka_behaviortree)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kmr_msgs REQUIRED)

set(library_name ${PROJECT_NAME})


set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  kmr_msgs
)

add_library(close_gripper_action_bt_node SHARED plugins/action/close_gripper.cpp)
list(APPEND plugin_libs close_gripper_action_bt_node)

add_library(open_gripper_action_bt_node SHARED plugins/action/open_gripper.cpp)
list(APPEND plugin_libs open_gripper_action_bt_node)

add_library(move_manipulator_action_bt_node SHARED plugins/action/move_manipulator.cpp)
list(APPEND plugin_libs move_manipulator_action_bt_node)

add_library(plan_manipulator_path_action_bt_node SHARED plugins/action/plan_manipulator_path.cpp)
list(APPEND plugin_libs plan_manipulator_path_action_bt_node)

add_library(object_search_action_bt_node SHARED plugins/action/object_search.cpp)
list(APPEND plugin_libs object_search_action_bt_node)

add_library(frame_empty_condition_bt_node SHARED plugins/condition/frame_empty.cpp)
list(APPEND plugin_libs frame_empty_condition_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


add_executable(behavior_tree_node2 src/behavior_tree_node2.cpp)
add_library(engine SHARED src/behavior_tree_engine.cpp)
ament_target_dependencies(behavior_tree_node2 ${dependencies})
target_link_libraries(behavior_tree_node2 engine)
ament_target_dependencies(engine ${dependencies})



install(TARGETS behavior_tree_node2 engine
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch plugins behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)
ament_export_dependencies(${dependencies})
ament_package()
