#include <chrono>
#include <memory>
#include <fstream>
#include <streambuf>
#include <string>
#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kuka_behaviortree/behavior_tree_engine.hpp"
#include "kmr_msgs/action/open_gripper.hpp"


static const rclcpp::Logger LOGGER = rclcpp::get_logger("behavior_tree_node");

class BehaviorTreeNode : public rclcpp::Node
{
public:
BehaviorTreeNode()
: Node("behavior_tree_node")
{
  RCLCPP_INFO(LOGGER, "Initialize node!");
  publisher_ = this->create_publisher<std_msgs::msg::String>("pub_topic", 10);
  start_subscriber_ = this->create_subscription<std_msgs::msg::String>("start_topic", 10,std::bind(&BehaviorTreeNode::start_callback, this, std::placeholders::_1));
  
  declare_parameter("bt_xml_filename");
  declare_parameter("plugin_lib_names");

  plugin_lib_names_ = get_parameter("plugin_lib_names").as_string_array();
  // plugin_lib_names_ = get_parameter("plugin_lib_names").as_string_array();
  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<kmr_behavior_tree_engine::BehaviorTreeEngine>(plugin_lib_names_);

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r", std::string("__node:=") + get_name() + "_client_node",
      "--"});

  RCLCPP_INFO(LOGGER,get_name());
  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);
  self_client_ = rclcpp_action::create_client<kmr_msgs::action::OpenGripper>(client_node_, "open_gripper");

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_); // NOLINT
  blackboard_->set<bool>("path_updated", false);
  blackboard_->set<bool>("initial_pose_received", false);
  
  std::string bt_xml_filename;
  get_parameter("bt_xml_filename", bt_xml_filename);


  std::ifstream xml_file(bt_xml_filename);
  if (!xml_file.good()) {
    RCLCPP_ERROR(LOGGER, "Couldn't open input XML file: %s", bt_xml_filename.c_str());
  }

  xml_string_ = std::string(std::istreambuf_iterator<char>(xml_file),std::istreambuf_iterator<char>());

  RCLCPP_DEBUG(LOGGER, "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_DEBUG(LOGGER, "Behavior Tree XML: %s", xml_string_.c_str());

  // Create the Behavior Tree from the XML input
  tree_ = bt_->buildTreeFromText(xml_string_, blackboard_);
  RCLCPP_INFO(LOGGER, "BehaviorTree successfully created");

}

private:
  void start_callback(std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(LOGGER, "I heard: '%s'", msg->data.c_str());
    if (msg->data == "OK"){

    }


  }


  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_subscriber_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::string xml_string_;
  std::unique_ptr<kmr_behavior_tree_engine::BehaviorTreeEngine> bt_;
  std::vector<std::string> plugin_lib_names_;

  // A client that we'll use to send a command message to our own task server
  rclcpp_action::Client<kmr_msgs::action::OpenGripper>::SharedPtr self_client_;

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;
};




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorTreeNode>());
  rclcpp::shutdown();

  return 0;
}
