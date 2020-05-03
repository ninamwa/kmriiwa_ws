#include "behaviortree_cpp_v3/condition_node.h"

namespace kmr_behavior_tree
{

class EmptyFrameCondition : public BT::ConditionNode
{
public:
  EmptyFrameCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
  {
  }

  EmptyFrameCondition() = delete;

  ~EmptyFrameCondition()
  {
    cleanup();
  }

  BT::NodeStatus tick() override
  {
   if (isFrameEmpty()) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  bool
  isFrameEmpty()
  {
    std::string check_frame;
    bool bb_frame;
    getInput("check_frame", check_frame);
    bb_frame = config().blackboard->get<bool>(check_frame);

    if (bb_frame) {
      setOutput("empty_frame", check_frame);
      return true;
    } else {
      return false;
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("check_frame", "Which frame to check if empty"),
      BT::OutputPort<std::string>("empty_frame", "Which frame found to be empty"),
    };
  }

protected:
  void cleanup()
  {
  }

private:

};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<kmr_behavior_tree::EmptyFrameCondition>("EmptyFrame");
}
