## 1. Description

This package is used for performing realtime SLAM and creating a map together with the KMR iiwa. 
Both the packages Cartographer and RTAB-Map can be used.

## 2. Requirements
The following packages needs to be installed:
- Cartographer
- Cartographer_ros
- RTAB-Map
- rtabmap_ros (ROS wrapper for RTAB-Map)
- Nav2_map_server (for saving the maps - it is a part of the Navigation2 package)



## 3. Run

To launch Cartographer, run: 
```
$ ros2 launch kmr_slam cartographer.launch.py
```
To launch RTAB-Map, run: 
```
$ ros2 launch kmr_slam rtabmap.launch.py 
```

The communication with the KMR must be started, and the robot can be driven around manually by using the implemented keyboard: 
```
$ ros2 run kmr_navigation2 twist_keyboard.py 
```

If you want to save the map which are created, this can be done by running the following command in a separate terminal:

```
$ ros2 run nav2_map_server map_saver
```
