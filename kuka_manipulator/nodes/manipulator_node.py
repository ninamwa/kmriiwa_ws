#!/usr/bin/env python3

import _thread as thread
import time
import os
import sys
import math
import rclpy
from rclpy.node import Node
import socket
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from kuka_manipulator.action import OpenGripper, CloseGripper, ObjectSearch
from kuka_communication.msg import LbrStatusdata
from functools import partial


def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class ManipulatorNode(Node):
    def __init__(self):
        super().__init__('manipulator_node')
        self.name='manipulator_node'

        # Make Publishers for relevant data
        #self.pub_realsense_search = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        #self.send_static_transform()

        sub_lbrstatus = self.create_subscription(LbrStatusdata,'/lbr_statusdata',self.lbr_statusdata_callback, qos_profile_sensor_data)
        self.opengripper_action_client = ActionClient(self, OpenGripper, 'open_gripper')
        self.closegripper_action_client = ActionClient(self, CloseGripper, 'close_gripper')
        self.camera_search_action_client = ActionClient(self, ObjectSearch, 'camera_object_search')

        i=0
        self.send_open_gripper_goal()
        #while True:
        #    time.sleep(0.1)
        #    if i == 50:
                #self.send_open_gripper_goal()
        
        #    if i <51:
        #        i=i+1
                #print(i)

        #thread.start_new_thread(self.run, ())

    #def run(self):
    #    while rclpy.ok() and self.soc.isconnected:
    #        if True:
    #           self.scan_callback(self.pub_laserscan1, self.soc.laserScanB1.pop(0))

    def lbr_statusdata_callback(self,data):
        if data.PathFinished == True:
            #self.send_close_gripper_goal()
            #self.send_open_gripper_goal()
            t=0
    def feedback_callback(self):
        print(0)

    def send_open_gripper_goal(self):
        goal_msg = OpenGripper.Goal()
        self.opengripper_action_client.wait_for_server()
        self._send_goal_future = self.opengripper_action_client.send_goal_async(goal_msg)
        print('HEI')
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        print("Hei2")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        """ GripperOpen = result.success
        GripperClose = not result.success
        print(GripperClose)
        print(GripperOpen) """

    def get_result_callback2(self, future):
        result = future.result().result
        GripperClose = result.success
        GripperOpen = not result.success
        print(GripperClose)
        print(GripperOpen)


    def scan_callback(self, publisher, values):
        if (len(values) == 4 and values[1] != self.last_scan_timestamp):
            kuka_timestamp = values[1]
            self.last_scan_timestamp =kuka_timestamp
            publisher.publish()


    def getTimestamp(self,nano):
        t = nano * 10 ** -9
        timestamp = Time()
        timestamp.sec = math.floor(t)
        timestamp.nanosec = int((t - timestamp.sec) * 10 ** 9)
        return timestamp


def main(args=None):
    rclpy.init(args=args)
    manipulator_node = ManipulatorNode()
    #manipulator_node.send_open_gripper_goal()
    rclpy.spin(manipulator_node)

    try:
        manipulator_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")



if __name__ == '__main__':
    main()
