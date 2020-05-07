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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("behavior_tree_node");

class BehaviorTreeNode : public rclcpp::Node
{
public:
BehaviorTreeNode()
: Node("behavior_tree_node")
{
  RCLCPP_INFO(LOGGER, "Initialize node!");
  start_subscriber_ = this->create_subscription<std_msgs::msg::String>("start_topic", 10,std::bind(&BehaviorTreeNode::start_callback, this, std::placeholders::_1));
  initial_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_to_pose");


  const std::vector<std::string> plugin_libs = {
    "move_gripper_action_bt_node",
    "move_manipulator_action_bt_node",
    "plan_manipulator_path_action_bt_node",
    "object_search_action_bt_node",
    "empty_frame_condition_bt_node",
    "navigate_vehicle_bt_node",
  };
  
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("bt_xml_filename");
  declare_parameter("WS1.position");
  declare_parameter("WS1.orientation");
  declare_parameter("WS2.position");
  declare_parameter("WS2.orientation");
  declare_parameter("WS3.position");
  declare_parameter("WS3.orientation");
  declare_parameter("HOME.position");
  declare_parameter("HOME.orientation");
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
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_); // NOLINT
  blackboard_->set<bool>("carryarea1", true);
  blackboard_->set<bool>("carryarea2", true);
  blackboard_->set<bool>("carryarea3", true);
  blackboard_->set<std::string>("current_frame", "driveposition");
  
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

  // Publish initial pose to be HOME position
  std::string position;
  float orientation;
  get_parameter("HOME.position", position);
  get_parameter("HOME.orientation", orientation);
  
  std::string delimiter = ",";
  std::string x_token = position.substr(0, position.find(delimiter)); 
  position.erase(0, position.find(delimiter) + delimiter.length());
  std::string y_token = position.substr(0, position.find(delimiter)); 

  float x = std::stof(x_token); 
  float y = std::stof(y_token);

  geometry_msgs::msg::PoseWithCovarianceStamped initial;
  initial.header.frame_id = "map";
  geometry_msgs::msg::Point point;
  point.x = x;  
  point.y =y;
  geometry_msgs::msg::Quaternion quat;
  quat.w = orientation;
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
    bool res = initializeGoalPose();
    if (!res){
      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose = create_pose("HOME");
      bool nav_res = send_navigation_goal(goal_pose);
      if (nav_res){
        RCLCPP_INFO(LOGGER, "Navigated successfully home!");
        exit(0);
      }
      else{
        RCLCPP_INFO(LOGGER, "Failed to navigate back home!");
        exit(0);
      }
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
        //start_bt();
        exit(0);
        break;

      case kmr_behavior_tree::BtStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "Object found and handled - failed");
        //start_bt();
        exit(0);
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

      goal_station = goal_list.front();
      goal_list.erase(goal_list.begin()); 

      goal_pose = create_pose(goal_station);
      RCLCPP_INFO(get_logger(), "Starting BT with new goal");
      // Update the goal pose on the blackboard
      blackboard_->set("current_goalpose", goal_pose);
      return true;
    } else{
      return false;
    }
  }

  bool send_navigation_goal(geometry_msgs::msg::PoseStamped pose){
    auto is_action_server_ready = action_client_->wait_for_action_server(std::chrono::seconds(5));
      if (!is_action_server_ready) {
        RCLCPP_ERROR(LOGGER,"NavigateToPose action server is not available.");
        return false;
      }

      // Send the goal pose
      auto navigation_goal_ = nav2_msgs::action::NavigateToPose::Goal();
      navigation_goal_.pose = pose;

      // Enable result awareness by providing an empty lambda function
      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = [](auto) {};

      auto future_goal_handle = action_client_->async_send_goal(navigation_goal_, send_goal_options);
      if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle) != rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(LOGGER, "Send goal call failed");
        return false;
      }

      // Get the goal handle and save so that we can check on completion in the timer callback
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_handle_ = future_goal_handle.get();
      if (!navigation_goal_handle_) {
        RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
        return false;
      }

      // Wait for the server to be done with the goal
      auto result_future = action_client_->async_get_result(navigation_goal_handle_);

      RCLCPP_INFO(LOGGER, "Waiting for result");
      if (rclcpp::spin_until_future_complete(client_node_, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS){
        RCLCPP_ERROR(LOGGER, "get result call failed :(");
        return false;
      }

      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result = result_future.get();

      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          return true;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(LOGGER, "Goal was aborted");
          return false;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(LOGGER, "Goal was canceled");
          return false;
        default:
          RCLCPP_ERROR(LOGGER, "Unknown result code");
          return false;
      }
  }
  
  geometry_msgs::msg::PoseStamped create_pose(std::string goal_station){
    std::string position;
    float orientation;

    get_parameter(goal_station + "." + "position", position);
    get_parameter(goal_station + "." + "orientation", orientation);

    std::string delimiter = ",";
    std::string x_token = position.substr(0, position.find(delimiter)); 
    position.erase(0, position.find(delimiter) + delimiter.length());
    std::string y_token = position.substr(0, position.find(delimiter));


    float x = std::stof(x_token); 
    float y = std::stof(y_token);
    
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
    quat.w = orientation;

    p.pose.position = point;
    p.pose.orientation = quat;
    return p;
  }
       


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_publisher_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::string xml_string_;
  std::unique_ptr<kmr_behavior_tree::BehaviorTreeEngine> bt_;
  std::vector<std::string> plugin_lib_names_;
  std::vector<std::string> goal_list;
  rclcpp::Node::SharedPtr client_node_;
};




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorTreeNode>());
  rclcpp::shutdown();

  return 0;
}
