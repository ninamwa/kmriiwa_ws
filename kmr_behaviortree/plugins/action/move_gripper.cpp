
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
      RCLCPP_INFO(node_->get_logger(),"Gripper opened successfully");
      std::string current_frame;
      current_frame = config().blackboard->get<std::string>("current_frame");
      if (current_frame.compare("carryarea1") == 0 || current_frame.compare("carryarea2") == 0 || current_frame.compare("carryarea3") == 0 ){
        config().blackboard->set(current_frame, false);
      }
    }else{
      RCLCPP_INFO(node_->get_logger(),"Gripper closed successfully");
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