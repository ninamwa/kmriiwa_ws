## 1. Description

This package handles the communication with the KMR iiwa. 
There is one launch file which will launch all of the seven communication nodes: 
- kmp_commands_node
- kmp_odometry_node
- kmp_laserscan_node
- kmp_statusdata_node
- lbr_statusdata_node
- lbr_commands_node
- lbr_sensordata_node

The connection type (UDP/TCP) can be set in the launch parameters. 
The IP address to your computer should be set in the parameter file called bringup.yaml. 
In this file, you can also change the port number for each of the nodes. 


## 2. Run
To launch all of the communcation nodes, run: 
```
$ ros2 launch kmr_communication sunrise_communication.py 
```
