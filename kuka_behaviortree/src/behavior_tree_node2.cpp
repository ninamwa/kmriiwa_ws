#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>


#include "rclcpp/rclcpp.hpp"


static const rclcpp::Logger LOGGER = rclcpp::get_logger("behavior_tree_node");

class BehaviorTreeNode
{
public:


private:



};




int main(int argc, char * argv[])
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("behavior_tree_node", "", node_options);

  //BehaviorTreeNode demo(node);
  //std::thread run_demo([&demo]() {
  //  demo.init();
  //});

  rclcpp::spin(node);
  //run_demo.join();
  rclcpp::shutdown();

  return 0;
}
