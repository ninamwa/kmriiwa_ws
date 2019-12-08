## 1. Description

This package handles the use of the Cartographer package for realtime SLAM and creating a map together with the KMR iiwa. 

## 2. Requirements
The following packages needs to be installed:
- Cartographer
- Cartographer_ros




## 3. Run

As of now, the launch file is not correctly launching the cartographer nodes, and this needs to be launched on its own. 

You need four terminals where you are running the commands: 

```
$ ros2 run kuka_communication server_TCP.py 
```
```
$ ros2 launch kuka_cartographer cartographer.launch.py
```
```
$ ros2 run cartographer_ros cartographer_node -configuration_directory PATH_TO/config -configuration_basename kuka_2d.lua
```
```
$ ros2 run cartographer_ros occupancy_grid_node
```

This will launch the communication node for the KMR iiwa, start Rviz with the correct configurations and also the state publisher of the URDF, which makes it possible to view the robot in Rviz. The two last commands will launch the necessary Cartographer nodes. 

If you want to save the map which are created, this can be done by running the following command in a separate terminal:

```
$ ros2 run nav2_map_server map_saver -f ~/PATH_TO/created_maps
```

The PATH_TO needs to be changed based on where your project is saved. 
