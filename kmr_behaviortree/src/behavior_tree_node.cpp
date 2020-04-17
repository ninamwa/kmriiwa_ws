#include <chrono>
#include <memory>
#include <fstream>
#include <streambuf>
#include <string>
#include <vector>
#include <iostream>
#include <bits/stdc++.h>
#include <stdio.h> 

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kmr_behaviortree/behavior_tree_engine.hpp"
#include "kmr_msgs/action/open_gripper.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>


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
  initial_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  const std::vector<std::string> plugin_libs = {
    "close_gripper_action_bt_node",
    "open_gripper_action_bt_node",
    "move_manipulator_action_bt_node",
    "plan_manipulator_path_action_bt_node",
    "object_search_action_bt_node",
    "frame_empty_condition_bt_node",
    "navigation_bt_node",
  };
  
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("bt_xml_filename");
  declare_parameter("WS1.position");
  declare_parameter("WS1.orientation");
  declare_parameter("goal_list");

  
  

  goal_list = get_parameter("goal_list").as_string_array();
  plugin_lib_names_ = get_parameter("plugin_lib_names").as_string_array();
  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<kmr_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r", std::string("__node:=") + get_name() + "_client_node",
      "--"});

  RCLCPP_INFO(LOGGER,get_name());
  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);


  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("clientnode", client_node_); // NOLINT
  blackboard_->set<bool>("frame_1", true);
  blackboard_->set<bool>("frame_2", true);
  blackboard_->set<bool>("frame_3", true);
  blackboard_->set<std::string>("current_frame", "drive_frame");
  
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

  geometry_msgs::msg::PoseWithCovarianceStamped initial;
  initial.header.frame_id = "map";
  geometry_msgs::msg::Point point;
  point.x = -2.0;  
  geometry_msgs::msg::Quaternion quat;
  quat.w = 0.1;
  initial.pose.pose.position = point;
  initial.pose.pose.orientation = quat;

  initial_publisher_->publish(initial);
  RCLCPP_INFO(LOGGER, "Initial pose published");

}

private:
  void start_callback(std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(LOGGER, "Start BT tree: '%s'", msg->data.c_str());
    if (msg->data == "OK"){
      start_bt();
    }


  }

  void start_bt(){
    bool res;
    res = initializeGoalPose();
    if (!res){
      exit(1);
    }

    auto is_canceling = [this]() {
      return false;
        };


    auto on_loop = [&]() {
        };

    kmr_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling);
    bt_->haltAllActions(tree_.root_node);

    switch (rc) {
      case kmr_behavior_tree::BtStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Object found and handled - succeeded");
        start_bt();
        break;

      case kmr_behavior_tree::BtStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "Object found and handled - failed");
        break;

      case kmr_behavior_tree::BtStatus::CANCELED:
        RCLCPP_INFO(get_logger(), "Object found and handled - canceled");
        break;

      default:
        throw std::logic_error("Invalid status return from BT");
    }
  }

  bool initializeGoalPose(){
    if (!goal_list.empty()){
      std::string goal_station;
      geometry_msgs::msg::PoseStamped goal_pose;
      std::string position;
      float orientation;

      goal_station = goal_list.front();
      goal_list.erase(goal_list.begin()); 
      get_parameter(goal_station + "." + "position", position);
      get_parameter(goal_station + "." + "orientation", orientation);

      std::string delimiter = ",";
      std::string x_token = position.substr(0, position.find(delimiter)); 
      position.erase(0, position.find(delimiter) + delimiter.length());
      std::string y_token = position.substr(0, position.find(delimiter)); 

      float x = std::stof(x_token); 
      float y = std::stof(y_token);

      goal_pose = create_pose(x,y,orientation);
      RCLCPP_INFO(get_logger(), "Starting BT with new goal");
      // Update the goal pose on the blackboard
      blackboard_->set("current_goalpose", goal_pose);
      return true;
    } else{
      return false;
    }
  }
  
  geometry_msgs::msg::PoseStamped create_pose(float x, float y, float th){
    geometry_msgs::msg::PoseStamped p;
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Quaternion quat;
    p.header.frame_id = "map";
        
    point.x = x;
    point.y = y;
    point.z = 0.0;

    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = 0.0;
    quat.w = th;

    p.pose.position = point;
    p.pose.orientation = quat;
    return p;
  }
       



  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_publisher_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::string xml_string_;
  std::unique_ptr<kmr_behavior_tree::BehaviorTreeEngine> bt_;
  std::vector<std::string> plugin_lib_names_;
  std::vector<std::string> goal_list;

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
