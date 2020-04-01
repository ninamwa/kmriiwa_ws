#include "kuka_behaviortree/bt_action_node.hpp"
#include "kuka_manipulator/action/close_gripper.hpp"


namespace kmr_behavior_tree
{
class CloseGripperAction : public BtActionNode<kuka_manipulator::action::CloseGripper>
{
public:
  CloseGripperAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<kuka_manipulator::action::CloseGripper>(xml_tag_name, action_name, conf)
  {
  }

  void on_tick() override
  {
  }

  BT::NodeStatus on_success() override
  {
    std::string current_frame;
    current_frame = config().blackboard->get<std::string>("current_frame");
    config().blackboard->set(current_frame, false);
    return BT::NodeStatus::SUCCESS;
  }

};

}  // namespace kmr_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<kmr_behavior_tree::CloseGripperAction>(
        name, "close_gripper", config);
    };

  factory.registerBuilder<kmr_behavior_tree::CloseGripperAction>(
    "CloseGripper", builder);
}