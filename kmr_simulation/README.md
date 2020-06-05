## 1. Description

This package is used for simulating the robot in Gazebo. 

## 2. Requirements
The Gazebo software and the ROS packages for interfacing with Gazebo, called gazebo_ros_pkgs, must be installed. 


## 3. Run

To start up Gazebo, run: 

```
$ ros2 launch kmr_simulation gazebo.launch.py
```
In addition, you need to launch the robot_state_publisher:
```
$ ros2 launch kmr_bringup state_publisher.launch.py
```

This will launch a model of the robot in Gazebo, and it is possible to control it, by using the twist keyboard:
```
$ ros2 run kmr_navigation2 twist_keyboard.py
```

The keyboard will make the robot move around in the simulated environment. 
Other packages like SLAM and Navigation may also be used together with Gazebo! 
