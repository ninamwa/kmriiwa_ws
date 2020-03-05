// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/planning_scene.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

#include <boost/scoped_ptr.hpp>

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_to_pose");
class MoveItCppDemo
{
public:

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
moveit::planning_interface::MoveItCppPtr moveit_cpp_;
MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
    , trajectory_publisher_(node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          "/fake_joint_trajectory_controller/joint_trajectory", 1))
  {
  }

void visualizeTrajectory(const robot_trajectory::RobotTrajectory& trajectory)
  {
    moveit_msgs::msg::DisplayRobotState waypoint;
    const auto start_time = node_->now();
    for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
    {
      moveit::core::robotStateToRobotStateMsg(trajectory.getWayPoint(i), waypoint.state);
      const auto waypoint_time =
          start_time + rclcpp::Duration::from_seconds(trajectory.getWayPointDurationFromStart(i));
      const auto now = node_->now();
      if (waypoint_time > now)
        rclcpp::sleep_for(std::chrono::nanoseconds((waypoint_time - now).nanoseconds()));

      robot_state_publisher_->publish(waypoint);
    }
  }


void run()
{
  RCLCPP_INFO(LOGGER, "Initialize RobotModel");
  const std::string PLANNING_GROUP = "manipulator";
  robot_model_loader::RobotModelLoader robot_model_loader(node_,"robot_description",true);
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");


  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  if (!node_->get_parameter("planner_plugin", planner_plugin_name)){
    RCLCPP_FATAL_STREAM(LOGGER,"Could not find planner plugin name");
  }
  
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL_STREAM(LOGGER,"Exception while creating planning plugin loader ");
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model,node_, node_->get_namespace()))
      RCLCPP_FATAL_STREAM(LOGGER,"Could not initialize planner instance");
    RCLCPP_INFO_STREAM(LOGGER,"Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    RCLCPP_ERROR_STREAM(LOGGER,"Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                        << "Available plugins: " << ss.str());
  }


  // Pose Goal
  // ^^^^^^^^^
  RCLCPP_INFO(LOGGER, "Set Goal");
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_iiwa";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

 
  moveit_msgs::msg::Constraints pose_goal =kinematic_constraints::constructGoalConstraints("tool0", pose, tolerance_pose, tolerance_angle);

  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  RCLCPP_INFO(LOGGER, "Plan to goal");
  planning_interface::PlanningContextPtr context =planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  RCLCPP_INFO(LOGGER, "test");
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    RCLCPP_ERROR(LOGGER,"Could not compute plan successfully");
  }else{
    RCLCPP_INFO(LOGGER, "Planning succesful");
    moveit_msgs::msg::MotionPlanResponse response;
    res.getMessage(response);

    //moveit::robot_trajectory::RobotTrajectory robot_trajectory;
    //robot_trajectory.robot_model_ = response.
    visualizeTrajectory(*res.trajectory_);


    RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
    //moveit_msgs::msg::RobotTrajectory robot_trajectory;
    //response.trajectory.getRobotTrajectoryMsg(robot_trajectory);
    trajectory_publisher_->publish(response.trajectory.joint_trajectory);
  }
  
  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
/*   
  moveit_msgs::msg::DisplayTrajectory display_trajectory;

  Visualize the trajectory 
  

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  //visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  //visual_tools.trigger();
  display_publisher.publish(display_trajectory);*/

  /* Set the state in the planning scene to the final state of the last plan 
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  planner_instance.reset(); */

}
};


int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_to_pose", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}