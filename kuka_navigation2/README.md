## 1. Description

This package handles the use of the Navigation2 package for autonomous control of the KMR iiwa robot. 

## 2. Requirements
The following packages needs to be installed:
- Navigation2



## 3. Run


You need two terminals where you are running the commands: 

```
$ ros2 run kuka_communication client_TCP.py 
```
```
$ ros2 launch kuka_navigation2 navigation2.launch.py map:=$HOME/PATH_TO/map/lab.yaml
```


This will launch the communication node for communicating with the KMR iiwa, and the navigation stack. 
In Rviz, you need to set the inital pose of the robot by pressing the "2D Pose Estimate" button. 
The robot should get some time to localize itself. 
Drive it a around by using a keyboard. This can be launched by running:

```
$ ros2 run kuka_navigation2 twist_keyboard.py
```
This keyboard send velocity commands to the robot.
The robot can also be navigating by using a pose keyboard, where you are giving a desired pose of the robot. This can be launched by running: 

```
$ ros2 run kuka_navigation2 pose_keyboard.py
```

When you are ready to start the navigation, press the "Navigation2 Goal" button, and set the goal target in the map. 

The PATH_TO needs to be changed based on where your project is saved. 
