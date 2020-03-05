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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
    , trajectory_publisher_(node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          "/fake_joint_trajectory_controller/joint_trajectory", 1))
  {
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit::planning_interface::PlanningComponent arm("manipulator", moveit_cpp_);

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Create collision object, planning shouldn't be too easy
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_footprint";
    collision_object.id = "box";


    shape_msgs::msg::SolidPrimitive mobile_base;
    mobile_base.type = mobile_base.BOX;
    mobile_base.dimensions = { 1.08, 0.63, 0.085 };
    geometry_msgs::msg::Pose mobile_base_pose;
    mobile_base_pose.position.x = 0.0;
    mobile_base_pose.position.y = 0.15;
    mobile_base_pose.position.z = -0.04225;

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.05, 0.3, 0.1 };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = -0.05;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.6;

    shape_msgs::msg::SolidPrimitive box2;
    box2.type = box2.BOX;
    box2.dimensions = { 0.3, 0.05, 0.1 };

    geometry_msgs::msg::Pose box2_pose;
    box2_pose.position.x = 0.0;
    box2_pose.position.y = 0.15;
    box2_pose.position.z = 0.55;

    shape_msgs::msg::SolidPrimitive box3;
    box3.type = box3.BOX;
    box3.dimensions = { 0.05, 0.3, 0.1 };

    geometry_msgs::msg::Pose box3_pose;
    box3_pose.position.x = 0.0;
    box3_pose.position.y = 0.40;
    box3_pose.position.z = 0.6;

    //collision_object.primitives.push_back(mobile_base);
    //collision_object.primitive_poses.push_back(mobile_base);
    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.primitives.push_back(box2);
    collision_object.primitive_poses.push_back(box2_pose);
    //collision_object.primitives.push_back(box3);
    //collision_object.primitive_poses.push_back(box3_pose);
    collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      //scene->processCollisionObjectMsg(collision_object);
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


    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

    //* Print end-effector pose. Remember that this is in the model frame *//*
    RCLCPP_INFO(LOGGER,"Translation: %f, %f, %f " ,end_effector_state.translation()[0],end_effector_state.translation()[1],end_effector_state.translation()[2]);
    //RCLCPP_INFO(LOGGER,"Rotation: \n" << end_effector_state.rotation() << "\n");
    double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);
    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        RCLCPP_INFO(LOGGER,"Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    }
    else
    {
      RCLCPP_INFO(LOGGER,"Did not find IK solution");
    }



    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goal");
    //arm.setGoal("pose1");

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "base_iiwa";
    pose_msg.pose.position.x = 0.3;
    pose_msg.pose.position.y = 0.4;
    pose_msg.pose.position.z = 0.0;
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

    // Quaternion quat = ToQuaternion(0,0,0);
    // printf("%f,%f,%f,%f", quat.w,quat.x,quat.y,quat.z);
    // pose_msg.pose.orientation.w= quat.w;
    // pose_msg.pose.orientation.x = quat.x;
    // pose_msg.pose.orientation.y= quat.y;
    // pose_msg.pose.orientation.z = quat.z; 

    arm.setGoal(pose_msg,"tool0");

    // Run actual plan
    RCLCPP_INFO(LOGGER, "Plan to goal");
    const auto plan_solution = arm.plan();
    if (plan_solution)
    {
      visualizeTrajectory(*plan_solution.trajectory);

      RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
      moveit_msgs::msg::RobotTrajectory robot_trajectory;
      plan_solution.trajectory->getRobotTrajectoryMsg(robot_trajectory);
      trajectory_publisher_->publish(robot_trajectory.joint_trajectory);
    }
  }
struct Quaternion
{
    double w, x, y, z;
};

    Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
    {
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        Quaternion q;
        q.w = cy * cp * cr + sy * sp * sr;
        q.x = cy * cp * sr - sy * sp * cr;
        q.y = sy * cp * sr + cy * sp * cr;
        q.z = sy * cp * cr - cy * sp * sr;

        return q;
    };
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

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
};


int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    // Let RViz initialize before running demo
    // TODO(henningkayser): use lifecycle events to launch node
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}