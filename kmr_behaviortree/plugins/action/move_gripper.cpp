#include "kmr_behaviortree/bt_action_node.hpp"
#include "kmr_msgs/action/gripper.hpp"


namespace kmr_behavior_tree
{
class MoveGripperAction : public BtActionNode<kmr_msgs::action::Gripper>
{
public:
  MoveGripperAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<kmr_msgs::action::Gripper>(xml_tag_name, action_name, conf)
  {
  }

  void on_tick() override
  {
    getInput("action", action);
    goal_.action = action;
  }

  BT::NodeStatus on_success() override
  {
    if(action == "open"){
      std::string current_frame;
      current_frame = config().blackboard->get<std::string>("current_frame");
      if (current_frame.compare("carryarea1") == 0 || current_frame.compare("carryarea2") == 0 || current_frame.compare("carryarea3") == 0 ){
        config().blackboard->set(current_frame, false);
      }
    }
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("action", "The action the gripper should perform: open/close"),
      });
  }

  private:
    std::string action;
};

}  // namespace kmr_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<kmr_behavior_tree::MoveGripperAction>(
        name, "move_gripper", config);
    };

  factory.registerBuilder<kmr_behavior_tree::MoveGripperAction>(
    "MoveGripper", builder);
}