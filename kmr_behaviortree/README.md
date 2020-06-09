## 1. Description

This package includes functionality for using a behavior tree to create more complex functionality for the robot. 
The plugin folder consists of custom made condition and action nodes. These nodes can be put together in multiple ways to create a full tree. 
Different behavior trees are found in the behavior_trees folder, and which tree to use is specified in the launch file. 
The param file can be used to specify different information, like positions of workstations and a list of goals. 
The behaviortree will start over for every goal pose in the list. When the list is empty, the robot will navigate back to the home position/docking station.

All other nodes which the behavior tree nodes depends upon must be running for the behavior tree to be initalized. 

## 2. Requirements
The following packages needs to be installed:
- behaviortree_cpp_v3



## 3. Run


To launch the behaviortree, run: 

```
$ ros2 launch kmr_behaviortree bt.launch.py 
```
When the behavior tree is initialized, a message to the /start_topic must be sent for the execution of the tree to start. 
This is done to make sure everything is correctly set up before starting, and should be removed at a later point. 
```
$ ros2 topic pub /start_topic std_msgs/msg/String {'data: OK'} -1
```
