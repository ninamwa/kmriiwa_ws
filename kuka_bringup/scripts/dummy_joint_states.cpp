#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("dummy_joint_states");

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 20);

  rclcpp::WallRate loop_rate(50);

  sensor_msgs::msg::JointState msg;
    msg.name.push_back("front_left_wheel_joint");
    msg.name.push_back("front_right_wheel_joint");
    msg.name.push_back("back_left_wheel_joint");
    msg.name.push_back("back_right_wheel_joint");
    msg.position.push_back(0.0);
    msg.position.push_back(0.0);
    msg.position.push_back(0.0);
    msg.position.push_back(0.0);



  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  auto counter = 0.0;
  auto joint_value = 0.0;
  //auto joint_value_2 = 0.0;

  while (rclcpp::ok()) {
    counter += 0.1;
    joint_value = counter;
    //joint_value_2 = std::sin(counter);

    for (size_t i = 0; i < msg.name.size(); ++i) {
      msg.position[i] = joint_value;
    }

    msg.header.stamp = clock->now();

    joint_state_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
