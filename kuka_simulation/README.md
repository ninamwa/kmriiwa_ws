## 1. Description

This package is used for simulating the robot in Gazebo. 

## 2. Requirements
To use this package, all gazebo files should be installed. This can be done by running: 
sudo apt install ros-dashing-gazebo-*

```
$ sudo apt install ros-dashing-gazebo-*
```



## 3. Run

To start up Gazebo, run: 

```
$ ros2 launch kuka_simulation gazebo.launch.py
```
In addition, you need to launch the robot_state_publisher:
```
$ ros2 launch kuka_bringup state_publisher.launch.py
```

This will launch a model of the robot in Gazebo, and it is possible to control it, by using the twist keyboard:
In addition, you need to launch the robot_state_publisher:
```
$ ros2 run kuka_navigation2 twist_keyboard.py
```

The keyboard will make the robot move around in the simulated environment. 
