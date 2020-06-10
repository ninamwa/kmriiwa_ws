
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
