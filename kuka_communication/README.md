## 1. Description

This package handles the communication with the KMR iiwa. 
There are two different versions implemented: one with UDP connection and one with TCP connection. 
The only difference between the files are which type of connection is implemented. 



## 2. Run
To run the kmr_communication_node with TCP connection: 
```
$ ros2 run kuka_communication client_TCP.py 
```

To run the kmr_communication_node with UDP connection: 
```
$ ros2 run kuka_communication client_UDP.py 
```

If you want to test any of the code without connecting to the robot a script which publishes dummy odometry and dummy laser data is created. To launch this, run: 

```
$ ros2 run kuka_communication dummy_data.py 
```
