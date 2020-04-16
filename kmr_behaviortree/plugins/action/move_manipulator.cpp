#include "kmr_behaviortree/bt_action_node.hpp"
#include "kmr_msgs/action/move_manipulator.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"


namespace kmr_behavior_tree
{
class MoveManipulatorAction : public BtActionNode<kmr_msgs::action::MoveManipulator>
{
public:
  MoveManipulatorAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<kmr_msgs::action::MoveManipulator>(xml_tag_name, action_name, conf)
  {
  }

  void on_tick() override
  {
    getInput("path", goal_.path);
    getInput("move_to_frame", current_frame);
  }

  BT::NodeStatus on_success() override
  {
    config().blackboard->set("current_frame", current_frame);
    return BT::NodeStatus::SUCCESS;
  }


  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<trajectory_msgs::msg::JointTrajectory>("path", "The path MoveIt has planned for the manipulator"),
        BT::InputPort<std::string>("move_to_frame", "The frame the manipulator is currently moving to"),
      });
  }
  private:
    std::string current_frame;
};

}  // namespace kmr_behavior_tree


//register our custom TreeNodes into the BehaviorTreeFactory
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<kmr_behavior_tree::MoveManipulatorAction>(
        name, "move_manipulator", config);
    };

  factory.registerBuilder<kmr_behavior_tree::MoveManipulatorAction>(
    "MoveManipulator", builder);
}