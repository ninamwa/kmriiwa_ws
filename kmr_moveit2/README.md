## 1. Description

This package handles the use of the MoveIt2 package for path planning for the LBR iiwa manipulator. 

## 2. Requirements
The following packages needs to be installed:
- MoveIt2



## 3. Run


Run the following command to launch MoveIt: 

```
$ ros2 launch kmr_moveit2 moveit.launch.py 
```

MoveIt can be used in three different ways: 
- Through the action PlanToFrame. 
- By publishing a ROS PoseStamped to the /moveit/goalpose topic
- By publishing a ROS String describing a configured frame to the /moveit/frame2 topic. The possible frames are described in the SRDF file.  
