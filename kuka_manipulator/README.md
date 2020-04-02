## 1. Description

This package control the LBR iiwa manipulator and handles the following components that are used with the robot:
- Robotiq 2F-85 Adaptive Gripper
- Intel RealSense D435 Depth Camera

## 2. Requirements
The following packages, with all the required dependencies defined at the corresponding repositories, must be installed:
- ros2_openvino_toolkit (eloquent branch)
- ros2_object_analytics (eloquent branch)
- ros2_intel_realsense (refactor branch)

Versions:
- Intel OpenVino Toolkit (Binary installation 2019R3)
- OpenCV (3.4 or later)

## 3. Run

The object detection with input from the D435 camera and the deep learning model can be launched by:

```
$ ros2 launch kuka_manipulator object_detection.launch.py
```
Detected objects of the type defined in the model are published to /object_analytics/localization which include:
Type: object_analytics_msgs/msg/ObjectsInBoxes3D
- Object:
    name
    probability
- ROI
    x_offset
    y_offset
    height
    width
- 3D bounding box
    min: x, y ,z
    max: x, y ,z

NB! The path to the model specified in param/object_detection.yaml may be incorrect. The path must be defined as the absolute path of the file in your directory.
