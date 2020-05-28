## 1. Description

This package handles the use of the Navigation2 package for autonomous control of the KMR iiwa robot. 

## 2. Requirements
The following packages needs to be installed:
- Navigation2



## 3. Run


You need two terminals where you are running the commands: 

```
$ ros2 launch kmr_communication sunrise_communication.launch.py 
```
```
$ ros2 launch kmr_navigation2 navigation2.launch.py
```


This will launch the communication nodes for communicating with the KMR iiwa, and the navigation stack. 
In Rviz, you need to set the inital pose of the robot by pressing the "2D Pose Estimate" button. 
The robot should get some time to localize itself. 
Drive it a around by using a keyboard. This can be launched by running:

```
$ ros2 run kmr_navigation2 twist_keyboard.py
```
This keyboard send velocity commands to the robot.
The robot can also be navigating by using a pose keyboard, where you are giving a desired pose of the robot. This can be launched by running: 

```
$ ros2 run kmr_navigation2 pose_keyboard.py
```

When you are ready to start the navigation, press the "Navigation2 Goal" button, and set the goal target in the map. 


The robot can also be navigating by using a pose keyboard, where you are giving a desired pose of the robot. This can be launched by running: 

A keyboard for manually controlling both the mobile vehicle and manipulator at the same time can be launched by running:
```
$ ros2 run kmr_navigation2 keyboard.py
```

