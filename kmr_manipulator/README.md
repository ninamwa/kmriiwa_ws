## 1. Description

This package handles tasks associated with manipulation of the LBR iiwa.

- Vision using a Intel® RealSense™ D435 camera
- Object detection and localization
- Grasping using a Robotiq 2F-85 gripping

## 2. Requirements
The following packages needs to be installed:
- ROS2 Intel Realsense
- ROS2 Openvino Toolkit 
- ROS2 Object Analytics

## 3. Run
The camera and gripper must be connected to a computer by USB. An onboard computer with ROS2 installed is useful for this puprose. 
The nodes are launched by running the command:

```
$ ros2 launch kmr_maniupulator nuc.launch.py 
```

