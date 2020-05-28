## 1. Description

This package handles the different files for bringing up the robot and showing it in Rviz. 

## 2. Requirements
The following packages needs to be installed:
- joint_state_publisher
- robot_state_publisher




## 3. Run

To visualize the URDF model of the robot in Rviz, you need two terminals and run the following commands:

```
$ ros2 launch kuka_bringup rviz.launch.py
```
```
$ ros2 launch kuka_bringup state_publisher.launch.py
```

This will automtically launch the dummy joint state publisher with a user interface, which do it possible to control the state of the joints in rviz. 
There do also exist a script for controlling the states. This can be launched by uncommenting the node in the *rviz.launch.py* file 
