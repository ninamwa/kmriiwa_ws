# kmriiwa_ws

Repository for master thesis and specialization project in Robotics & Automation, at NTNU Fall 2019, by Charlotte Heggem and Nina Marie Wahl. 

**Title:** Configuration and Control of KMR iiwa with ROS2

**Intention:**
This project aims to create a communication API between a KUKA robot, KMR iiwa, and ROS2. 
Multiple ROS packages are used for including functionality. 
Navigation2 is used for navigating the mobile vehicle. 
Cartographer and RTAB-Map is used for SLAM. 
MoveIt2 is used for path planning for the manipulator. 


Multiple Intel Realsens D435 cmeras are used to provide better moving and detection of objects. 
A Robotiq gripper is used for picking up objects. 
The cameras and gripper are launched at a separate onboard computer (Intel NUC). 

**System requirements:** 

- Ubuntu 18.04.3
- Python 3.6.9
- ROS Eloquent


**Required ROS Packages:**
- Gazebo packages
- Navigation2
- MoveIt2
- Cartographer
- RTAB-Map ROS wrapper (dependent on RTAB-Map)
- Ros2 Intel Realsense (ROS2 Wrapper for Intel® RealSense™ Devices)
- ROS2 Openvino Toolkit (dependent on OpenVino Toolkit)
- ROS2 Object Analytics
