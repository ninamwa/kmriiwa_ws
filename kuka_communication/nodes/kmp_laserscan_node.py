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
from rclpy.utilities import remove_ros_args
import argparse

from script.tcpSocket import TCPSocket
from script.udpSocket import UDPSocket


def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class KmpLaserScanNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('kmp_laserscan_node')
        self.name='kmp_laserscan_node'
        self.declare_parameter('port')
        port = int(self.get_parameter('port').value)
        if robot == 'KMR1':
            self.declare_parameter('/KMR1/ip')
            ip = str(self.get_parameter('/KMR1/ip').value)
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


        self.last_scan_timestamp = 0


        # Make Publishers for relevant data
        self.pub_laserscan1 = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.pub_laserscan4 = self.create_publisher(LaserScan, 'scan_2', qos_profile_sensor_data)
        self.send_static_transform()


        while not self.soc.isconnected:
            pass
        self.get_logger().info('Node is ready')

        thread.start_new_thread(self.run, ())

    def run(self):
        while rclpy.ok() and self.soc.isconnected:
            if len(self.soc.laserScanB1):
                self.scan_callback(self.pub_laserscan1, self.soc.laserScanB1.pop(0))
            if len(self.soc.laserScanB4):
                self.scan_callback(self.pub_laserscan4, self.soc.laserScanB4.pop(0))



    def scan_callback(self, publisher, values):
        if (len(values) == 4 and values[1] != self.last_scan_timestamp):
            kuka_timestamp = values[1]
            self.last_scan_timestamp =kuka_timestamp
            scan = LaserScan()
            scan.header.stamp = self.getTimestamp(self.get_clock().now().nanoseconds)
            if values[2] == '1801':
                scan.header.frame_id = "scan"
            elif values[2] == '1802':
                scan.header.frame_id="scan_2"
            scan.angle_increment = (0.5*math.pi)/180
            scan.angle_min = (-135*math.pi)/180
            scan.angle_max = (135*math.pi)/180
            scan.range_min = 0.12 # disse må finnes ut av
            scan.range_max = 3.5 # finn ut
            try:
                scan.ranges = [float(s) for s in values[3].split(',') if len(s)>0]
            except ValueError as e:
                print(values[3].split(','))
                print("Error", e)

            if len(scan.ranges) == 541:
                publisher.publish(scan)
            else:
                #print(len(scan.ranges)) ##FOR DEBUGGING
                t=0


    def getTimestamp(self,nano):
        t = nano * 10 ** -9
        timestamp = Time()
        timestamp.sec = math.floor(t)
        timestamp.nanosec = int((t - timestamp.sec) * 10 ** 9)
        return timestamp


    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]

    def send_static_transform(self):
        broadcaster1 = StaticTransformBroadcaster(self)
        broadcaster2 = StaticTransformBroadcaster(self)
        static_transformStamped = TransformStamped()
        static_transformStamped.header.frame_id = "laser_B4_link"
        static_transformStamped.child_frame_id = "scan_2"
        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0
        quat = self.euler_to_quaternion(0, 0, 0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
        broadcaster1.sendTransform(static_transformStamped)
        static_transformStamped.header.frame_id = "laser_B1_link"
        static_transformStamped.child_frame_id = "scan"
        broadcaster2.sendTransform(static_transformStamped)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))
    rclpy.init(args=argv)
    laserscan_node = KmpLaserScanNode(args.connection,args.robot)

    rclpy.spin(laserscan_node)

    #while rclpy.ok():
    #    rclpy.spin_once(odometry_node)
    try:
        laserscan_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")



if __name__ == '__main__':
    main()
