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

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 20);

  rclcpp::WallRate loop_rate(50);

  sensor_msgs::msg::JointState msg;
    msg.name.push_back("joint_a1");
    msg.name.push_back("joint_a2");
    msg.name.push_back("joint_a3");
    msg.name.push_back("joint_a4");
    msg.name.push_back("joint_a5");
    msg.name.push_back("joint_a6");
    msg.name.push_back("joint_a7");
    //msg.position.push_back(0.0);
    //msg.position.push_back(1.0);
    //msg.position.push_back(2.0);
    //msg.position.push_back(0.0);
    //msg.position.push_back(0.0);
    //msg.position.push_back(0.0);
    //msg.position.push_back(0.0);

    msg.position.push_back(-1.5810079050729513);
    msg.position.push_back(0.8167376905451679);
    msg.position.push_back(-0.004375380628040148);
    msg.position.push_back(-1.5857193233230227);
    msg.position.push_back(-7.76584250674877e-06);
    msg.position.push_back(0.7069321699035735);
    msg.position.push_back(-0.8036188590817025);

    /* msg.effort.push_back(1.1856913528442383);
    msg.effort.push_back(64.26652892303467);
    msg.effort.push_back(0.9920562973022461);
    msg.effort.push_back(-18.452515979766847);
    msg.effort.push_back(0.407613582611084);
    msg.effort.push_back(-0.045390228271484374);
    msg.effort.push_back(0.2566550407409668); */


  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  auto counter = 0.0;
  auto joint_value = 0.0;
  //auto joint_value_2 = 0.0;

  /* std::cout << "hei" << std::endl;
  joint_state_pub->publish(msg);
  msg.position[1]=-1.5810079050729513;
  msg.position[2]=0.8167376905451679;
  msg.position[3]=-0.004375380628040148;
  msg.position[4]=-1.5857192034407195;
  msg.position[5]=-7.76584250674877e-06;
  msg.position[6]=0.7069319901346606;
  msg.position[7]=-0.8036188590817025;
  joint_state_pub->publish(msg);
  msg.position[1]=-1.5810079050729513;
  msg.position[2]=0.8167376905451679;
  msg.position[3]=-0.004375440514650108;
  msg.position[4]=-1.5857193233230227;
  msg.position[5]=-7.76584250674877e-06;
  msg.position[6]=0.7069319901346606;
  msg.position[7]=-0.8036188590817025;
  joint_state_pub->publish(msg);
  msg.position[1]=-1.5810079649595612;
  msg.position[2]=0.8167376905451679;
  msg.position[3]=-0.004375380628040148;
  msg.position[4]=-1.5857193233230227;
  msg.position[5]=-7.76584250674877e-06;
  msg.position[6]=0.7069319901346606;
  msg.position[7]=-0.8036188590817025;
  joint_state_pub->publish(msg);
  msg.position[1]=-1.5810079050729513;
  msg.position[2]=0.8167376905451679;
  msg.position[3]=-0.004375380628040148;
  msg.position[4]=-1.5857193233230227;
  msg.position[5]=-7.76584250674877e-06;
  msg.position[6]=0.7069319901346606;
  msg.position[7]=-0.8036188590817025;
  joint_state_pub->publish(msg);
  msg.position[1]=-1.5810079050729513;
  msg.position[2]=0.8167376905451679;
  msg.position[3]=-0.0043753206323471116;
  msg.position[4]=-1.5857192034407195;
  msg.position[5]=-7.76584250674877e-06;
  msg.position[6]=0.7069319901346606;
  msg.position[7]=-0.8036188590817025;
  joint_state_pub->publish(msg);
  msg.position[1]=-1.5810079050729513;
  msg.position[2]=0.8167376905451679;
  msg.position[3]=-0.004375440514650108;
  msg.position[4]=-1.5857192034407195;
  msg.position[5]=-7.76584250674877e-06;
  msg.position[6]=0.7069319901346606;
  msg.position[7]=-0.8036188590817025;
  joint_state_pub->publish(msg);
  msg.position[1]=-1.5810079050729513;
  msg.position[2]=0.8167376905451679;
  msg.position[3]=-0.004375380628040148;
  msg.position[4]=-1.5857192034407195;
  msg.position[5]=-7.76584250674877e-06;
  msg.position[6]=0.7069319901346606;
  msg.position[7]=-0.8036188590817025;
  joint_state_pub->publish(msg);
  msg.position[1]=-1.5810079050729513;
  msg.position[2]=0.8167376905451679;
  msg.position[3]=-0.004375380628040148;
  msg.position[4]=-1.5857193233230227;
  msg.position[5]=-7.76584250674877e-06;
  msg.position[6]=0.7069319901346606;
  msg.position[7]=-0.8036188590817025;
  joint_state_pub->publish(msg);
  msg.position[1]=-1.5810079649595612;
  msg.position[2]=0.8167377504317779;
  msg.position[3]=-0.004375380628040148;
  msg.position[4]=-1.5857192034407195;
  msg.position[5]=-7.957479658617746e-06;
  msg.position[6]=0.7069319901346606;
  msg.position[7]=-0.8036188590817025;
  joint_state_pub->publish(msg);
  rclcpp::spin_some(node); */

  while (rclcpp::ok()) {
    counter += 0.000001;
    joint_value = counter;
    //joint_value_2 = std::sin(counter);


    //for (size_t i = 0; i < msg.name.size(); ++i) {
    //  msg.position[i] = msg.position[i] +counter;
    //}
    //msg.position[3] = msg.position[3] +counter;

    msg.header.stamp = clock->now();

    joint_state_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  } 

  rclcpp::shutdown();

  return 0;
}
