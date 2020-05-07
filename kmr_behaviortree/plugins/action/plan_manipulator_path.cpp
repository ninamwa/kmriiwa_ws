#include "kmr_behaviortree/bt_action_node.hpp"
#include "kmr_msgs/action/plan_to_frame.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>

namespace kmr_behavior_tree
{
class PlanManipulatorPathAction : public BtActionNode<kmr_msgs::action::PlanToFrame>
{
public:
  PlanManipulatorPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<kmr_msgs::action::PlanToFrame>(xml_tag_name, action_name, conf)
  {
  }

  void on_tick() override
  {
    if (!getInput("plan_to_frame", plan_to_frame)) {
      RCLCPP_ERROR(node_->get_logger(),"PlanToFrameAction: frame not provided");
      return;
    }
    goal_.frame = plan_to_frame;
    RCLCPP_INFO(node_->get_logger(),"Start planning to %s", plan_to_frame.c_str());
    if (plan_to_frame == "object"){
      geometry_msgs::msg::PoseStamped pose_msg;
      getInput("object_pose",pose_msg);
      goal_.pose = pose_msg;
    }
  }

  BT::NodeStatus on_success() override
  {
    setOutput("manipulator_path", result_.result->path);
    setOutput("move_to_frame", plan_to_frame);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("plan_to_frame", "The frame MoveIt should plan to"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("object_pose", "Pose of the object manipulator should move to"),
        BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("manipulator_path", "The path MoveIt has planned for the manipulator"),
        BT::OutputPort<std::string>("move_to_frame", "Frame we should move to"),
      });
  }

  private:
    std::string plan_to_frame;
};

}  // namespace kmr_behavior_tree


//register our custom TreeNodes into the BehaviorTreeFactory
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<kmr_behavior_tree::PlanManipulatorPathAction>(
        name, "/moveit/frame", config);
    };

  factory.registerBuilder<kmr_behavior_tree::PlanManipulatorPathAction>(
    "PlanManipulatorPath", builder);
}