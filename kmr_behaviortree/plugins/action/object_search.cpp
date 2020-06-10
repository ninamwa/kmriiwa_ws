
// Copyright 2019 Nina Marie Wahl and Charlotte Heggem.
// Copyright 2019 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kmr_behaviortree/bt_action_node.hpp"
#include "kmr_msgs/action/object_search.hpp"
#include "iostream"


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
    RCLCPP_INFO(node_->get_logger(),"Start searching for object");
  }

  BT::NodeStatus on_success() override
  {
    setOutput("object_pose", result_.result->pose);
    RCLCPP_INFO(node_->get_logger(),"ObjectPose received");
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_aborted() override
  {
    RCLCPP_INFO(node_->get_logger(),"No object found");
    return BT::NodeStatus::FAILURE;
  }
  
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("object_pose", "Pose of object found by object search"),
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