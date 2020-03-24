/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser
   Desc: A simple demo node running MoveItCpp for planning and execution
*/
/* Author: Nina Marie Wahk
   Desc: Fuctionality added for communication with manipulator_node
*/


#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <std_msgs/msg/string.hpp>
#include <kuka_manipulator/action/drive_to_frame.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
    , trajectory_publisher_(node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          "/fake_joint_trajectory_controller/joint_trajectory", 1))
    , goal_pose_subscriber_(node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/moveit/goalpose", 10 ,std::bind(&MoveItCppDemo::goalpose_callback, this, std::placeholders::_1)))
    , frame_subscriber_(node_->create_subscription<std_msgs::msg::String>("/moveit/frame",10,std::bind(&MoveItCppDemo::frame_callback, this, std::placeholders::_1)))
    , action_server_(rclcpp_action::create_server<kuka_manipulator::action::DriveToFrame>(
      node_->get_node_base_interface(),
      node_->get_node_clock_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_waitables_interface(),
      "/moveit/frame",
      std::bind(&MoveItCppDemo::handle_goal, this,  std::placeholders::_1,  std::placeholders::_2),
      std::bind(&MoveItCppDemo::handle_cancel, this,  std::placeholders::_1),
      std::bind(&MoveItCppDemo::handle_accepted, this,  std::placeholders::_1)))
  {
  }

  void init()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    arm = std::make_shared<moveit::planning_interface::PlanningComponent>("manipulator", moveit_cpp_);

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Create collision object, planning shouldn't be too easy
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_footprint";
    collision_object.id = "box";


    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.05, 0.3, 0.1 };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = -0.05;
    box_pose.position.y = 0.0;
    box_pose.position.z = 1.0;

    shape_msgs::msg::SolidPrimitive box2;
    box2.type = box2.BOX;
    box2.dimensions = { 0.3, 0.05, 0.1 };

    geometry_msgs::msg::Pose box2_pose;
    box2_pose.position.x = 0.0;
    box2_pose.position.y = 0.15;
    box2_pose.position.z = 1.0;

    shape_msgs::msg::SolidPrimitive box3;
    box3.type = box3.BOX;
    box3.dimensions = { 0.05, 0.3, 0.1 };

    geometry_msgs::msg::Pose box3_pose;
    box3_pose.position.x = 0.0;
    box3_pose.position.y = 0.40;
    box3_pose.position.z = 1.0;

    shape_msgs::msg::SolidPrimitive snus;
    snus.type = snus.CYLINDER;
    snus.dimensions = {0.01, 0.03};

    geometry_msgs::msg::Pose snus_pose;
    snus_pose.position.x = -0.2;
    snus_pose.position.y = 0.0;
    snus_pose.position.z = 0.6;

  

    // collision_object.primitives.push_back(box);
    // collision_object.primitive_poses.push_back(box_pose);
    // collision_object.primitives.push_back(box2);
    // collision_object.primitive_poses.push_back(box2_pose);
    //collision_object.primitives.push_back(box3);
    //collision_object.primitive_poses.push_back(box3_pose);
    collision_object.primitives.push_back(snus);
    collision_object.primitive_poses.push_back(snus_pose);
    collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene
    robot_model_loader::RobotModelLoader robot_model_loader(node_,"robot_description",true);
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(LOGGER,"Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    //kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER,"Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  


    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goal");
    //arm.setGoal("pose1");

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "base_footprint";
    pose_msg.pose.position.x = -0.2;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.75;
    pose_msg.pose.orientation.w = 0.0;
    pose_msg.pose.orientation.x = 1.0;
    pose_msg.pose.orientation.y= 0.0;
    pose_msg.pose.orientation.z = 0.0;


    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_iiwa";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.4;
    pose.pose.position.z = 0.9;
    pose.pose.orientation.w = 1.0;

    //arm->setGoal(pose_msg,"gripper_base_link");
    arm->setGoal("home");
    MoveItCppDemo::move();
    rclcpp::sleep_for(std::chrono::nanoseconds(6000000000));
    
    moveit::core::RobotStatePtr start_state = moveit_cpp_->getCurrentState();
    start_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER,"Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    arm->setGoal(pose_msg, "gripper_base_link");
    MoveItCppDemo::move();
    rclcpp::sleep_for(std::chrono::nanoseconds(6000000000));

    // arm->setGoal("search_2");
    // MoveItCppDemo::move();

    // rclcpp::sleep_for(std::chrono::nanoseconds(6000000000));

    // arm->setGoal("search_3");
    // MoveItCppDemo::move();
    
  }

private:
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

  void frame_callback(std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(LOGGER, "Frame Received: %s", msg->data);
    arm->setGoal(msg->data);
    move();
  }

  void goalpose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
/*     ::planning_interface::MotionPlanRequest req;
    moveit::core::RobotStatePtr start_state = moveit_cpp_->getCurrentState();
    start_state->update();
    moveit::core::robotStateToRobotStateMsg(*start_state, req.start_state);
    {
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->setCurrentState(*start_state);
    } */
    RCLCPP_INFO(LOGGER, "GoalPose Received: %f, %f, %f", msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    arm->setGoal(*msg,"tool0");
    move();
    
  }

  void move(){
    RCLCPP_INFO(LOGGER, "Plan to goal");

    default_parameters.planning_attempts = 1;
    default_parameters.planning_time = 5.0;
    default_parameters.max_velocity_scaling_factor = 0.4;
    default_parameters.max_acceleration_scaling_factor = 0.4;

    planning_pipeline_names = moveit_cpp_->getPlanningPipelineNames("manipulator");
    if (!planning_pipeline_names.empty())
      default_parameters.planning_pipeline = *planning_pipeline_names.begin();
    const auto plan_solution = arm->plan(default_parameters);
    if (plan_solution)
    {
      visualizeTrajectory(*plan_solution.trajectory);

      RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
      moveit_msgs::msg::RobotTrajectory robot_trajectory;
      plan_solution.trajectory->getRobotTrajectoryMsg(robot_trajectory);
      trajectory_publisher_->publish(robot_trajectory.joint_trajectory);
    }
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const kuka_manipulator::action::DriveToFrame::Goal> goal)
  {
    RCLCPP_INFO(LOGGER, "Received request to move to: %s", goal->frame);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<kuka_manipulator::action::DriveToFrame>> goal_handle)
  {
    RCLCPP_INFO(LOGGER, "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<kuka_manipulator::action::DriveToFrame>> goal_handle)
  {
    std::thread{std::bind(&MoveItCppDemo::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

   void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<kuka_manipulator::action::DriveToFrame>> goal_handle)
  {
    RCLCPP_INFO(LOGGER, "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    
    auto result = std::make_shared<kuka_manipulator::action::DriveToFrame::Result>();

    arm->setGoal(goal->frame);
    RCLCPP_INFO(LOGGER, "Plan to goal");
    default_parameters.planning_attempts = 1;
    default_parameters.planning_time = 5.0;
    default_parameters.max_velocity_scaling_factor = 0.4;
    default_parameters.max_acceleration_scaling_factor = 0.4;
    planning_pipeline_names = moveit_cpp_->getPlanningPipelineNames("manipulator");
    if (!planning_pipeline_names.empty())
      default_parameters.planning_pipeline = *planning_pipeline_names.begin();
    const auto plan_solution = arm->plan(default_parameters);

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(LOGGER, "Goal Canceled");
      return;
    }

    if (plan_solution)
    {
      // Check if goal is done
      if (rclcpp::ok()) {
        result->success = true;
        result->frame = goal->frame;
        goal_handle->succeed(result);
        RCLCPP_INFO(LOGGER, "Goal Succeeded");
      }

      visualizeTrajectory(*plan_solution.trajectory);

      RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
      moveit_msgs::msg::RobotTrajectory robot_trajectory;
      plan_solution.trajectory->getRobotTrajectoryMsg(robot_trajectory);
      trajectory_publisher_->publish(robot_trajectory.joint_trajectory);
    }
    else{
        result->success = false;
        result->frame = goal->frame;
        goal_handle->abort(result);
        RCLCPP_INFO(LOGGER, "Goal Failed - Planning Failed");

    }
  
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr frame_subscriber_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  std::shared_ptr<moveit::planning_interface::PlanningComponent> arm;
  std::set<std::string> planning_pipeline_names;
  moveit::planning_interface::PlanningComponent::PlanRequestParameters default_parameters;
  rclcpp_action::Server<kuka_manipulator::action::DriveToFrame>::SharedPtr action_server_;
};


int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    rclcpp::sleep_for(std::chrono::seconds(3));
    demo.init();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}