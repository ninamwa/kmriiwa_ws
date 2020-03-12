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
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse
import numpy as np

from script.tcpSocket import TCPSocket
from script.udpSocket import UDPSocket


def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class LbrSensordataNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('lbr_sensordate_node')
        self.name='lbr_sensordate_node'
        self.declare_parameter('port')
        port = int(self.get_parameter('port').value)
        if robot == 'KMR1':
            self.declare_parameter('KMR1/ip')
            ip = str(self.get_parameter('KMR1/ip').value)
        elif robot == 'KMR2':
            self.declare_parameter('KMR2/ip')
            ip = str(self.get_parameter('KMR2/ip').value)
        else:
            ip = None

        if connection_type == 'TCP':
            self.soc = TCPSocket(ip, port,self.name)
        elif connection_type == 'UDP':
            self.soc = UDPSocket(ip, port,self.name)
        else:
            self.soc = None


        self.last_data_timestamp = 0


        # Make Publishers for relevant data
        self.pub_lbr_sensordata = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.joint_names = ["joint_a1","joint_a2","joint_a3","joint_a4","joint_a5","joint_a6","joint_a7"]

        while not self.soc.isconnected:
            pass
        self.get_logger().info('Node is ready')

        thread.start_new_thread(self.run, ())

    def run(self):
        while rclpy.ok() and self.soc.isconnected:
            if len(self.soc.lbr_sensordata):
                self.data_callback(self.pub_lbr_sensordata, self.soc.lbr_sensordata.pop(0))


    def data_callback(self, publisher, values):
        data=values[1]
        if (data.split(',')[1] != self.last_data_timestamp):
            self.last_data_timestamp = data.split(',')[1]
            effort = [float(s) for s in data.split('MeasuredTorque:')[1].split(',') if len(s)>0]
            position = [float(s) for s in data.split('JointPosition:')[1].split('MeasuredTorque:')[0].split(',') if len(s)>0]
            msg = JointState()
            msg.header.stamp = self.getTimestamp(self.get_clock().now().nanoseconds)
            msg.name = self.joint_names
            msg.position = position
            msg.effort = effort
            publisher.publish(msg)

    def getTimestamp(self,nano):
        t = nano * 10 ** -9
        timestamp = Time()
        timestamp.sec = math.floor(t)
        timestamp.nanosec = int((t - timestamp.sec) * 10 ** 9)
        return timestamp



def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))
    print(args)
    rclpy.init(args=argv)
    lbr_sensordata_node = LbrSensordataNode(args.connection,args.robot)

    rclpy.spin(lbr_sensordata_node)

    #while rclpy.ok():
    #    rclpy.spin_once(odometry_node)
    try:
        lbr_sensordata_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")



if __name__ == '__main__':
    main()
