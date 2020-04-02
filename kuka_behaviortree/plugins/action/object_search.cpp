#include "kuka_behaviortree/bt_action_node.hpp"
#include "kmr_msgs/action/object_search.hpp"


namespace kmr_behavior_tree
{
class ObjectSearchAction : public BtActionNode<kmr_msgs::action::ObjectSearch>
{
public:
  ObjectSearchAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<kmr_msgs::action::ObjectSearch>(xml_tag_name, action_name, conf)
  {
  }

  void on_tick() override
  {
  }

  BT::NodeStatus on_success() override
  {
    setOutput("ObjectPose", result_.result->pose);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<geometry_msgs::msg::Pose>("ObjectPose", "Pose of object found by object search"),
      });
  }
};

}  // namespace kmr_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<kmr_behavior_tree::ObjectSearchAction>(name, "object_search", config);
    };

  factory.registerBuilder<kmr_behavior_tree::ObjectSearchAction>("ObjectSearch", builder);
}