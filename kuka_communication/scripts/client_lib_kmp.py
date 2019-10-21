#!/usr/bin/env python
# KUKA API for ROS2
version = 'V012019'

# 2019 Charlotte Heggem charlheg@stud.ntnu.no
# NTNU

# This script generates a ROS2 node for comunicating with KUKA KMP iiwa
# Dependencies: conf.txt, ROS server, Rclpy, KUKA KMR iiwa java SDK, KUKA KMR iiwa robot.

#######################################################################################################################
import os
import rclpy
import time
from std_msgs.msg import String

class kuka_iiwa_ros_client:

    def __init__(self): # Makes kuka_iiwa ROS node
        self.JointPosition = ([None,None,None,None,None,None,None],None)

        #    Make a listener for all kuka_iiwa data
        rclpy.init(args=None)
        self.kuka_client = rclpy.create_node("kuka_iiwa_client")
        self.kuka_client.create_subscription(String, "JointPosition", self.JointPosition_callback, 10)

        #   Make Publishers for kuka_iiwa commands
        self.pub_kuka_command = self.kuka_client.create_publisher(String, "kuka_command",10)

        ## PRØVER MEG PÅ THREAD :D
        thread.start_new_thread(self.executor,())

    #   ~M: __init__ ==========================
    def executor(self):
        while rclpy.ok():
            rclpy.spin_once(self.kuka_client)
        self.kuka_client.destroy_node()
        rclpy.shutdown()


    def send_command(self, command_str):
        #rospy.loginfo(command_str)
        command = String()
        command.data  = command_str
        self.pub_kuka_command.publish(command)
        time.sleep(0.001)  # 1000 hz, should maybe be 100 hz?
        # CAN NOT BE CALLED WITHOUT THIS DELAY (Tested for smaller timeouts)

    #   M: callbacks ===========================
    #   Receiving command string and sending it to KUKA iiwa

    def JointPosition_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "Received JointPosition " + str(data.data) )
        # e.g. [0.0, 0.17, 0.0, 1.92, 0.0, 0.35, 0.0] 1459253274.1
        self.JointPosition = ([float(x) for x in data.data.split(']')[0][1:].split(', ')], float(data.data.split(']')[1]))

    

#   ~Class: Kuka iiwa ROS client    #####################
######################################################################################################################
