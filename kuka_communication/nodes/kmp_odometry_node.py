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
from scripts.TCPSocket import TCPSocket
from scripts.UDPSocket import UDPSocket

from rclpy.utilities import remove_ros_args
import argparse

def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class KmpOdometryNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('kmp_odometry_node')

        if robot == 'KMR1':
            port = 30001
            ip = 1010
        elif robot == 'KMR2':
            port = 1223
            ip= 1212
        else:
            port=None
            ip=None


        if connection_type == 'TCP':
            self.soc = TCPSocket(ip,port)
        elif connection_type == 'UDP':
            self.soc=UDPSocket(ip,port)
        else:
            self.soc=None

        self.last_odom_timestamp = 0

        # Make Publisher for odometry
        self.pub_odometry = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)

        # Create tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        while not self.soc.isconnected:
            pass
        print('Ready to start')

        thread.start_new_thread(self.run, ())

    def run(self):
        while rclpy.ok() and self.soc.isconnected:
            self.odom_callback(self.pub_odometry, self.soc.odometry)

    def odom_callback(self, publisher, values):
        if (len(values) == 8 and values[1] != self.last_odom_timestamp):
            kuka_timestamp = values[1]
            self.last_odom_timestamp = kuka_timestamp

            x = float(values[2].split(":")[1])
            y = float(values[3].split(":")[1])
            th = float(values[4].split(":")[1])
            vx = float(values[5].split(":")[1])
            vy = float(values[6].split(":")[1])
            vth = float(values[7].split(":")[1])


            odom = Odometry()
            odom.header.stamp = self.getTimestamp(self.kmp_odometry_node.get_clock().now().nanoseconds)
            odom.header.frame_id = "odom"

            point = Point()
            point.x = x
            point.y = y
            point.z = float(0)

            odom_quat = Quaternion()
            quat = self.euler_to_quaternion(0,0,th)
            odom_quat.x = quat[0]
            odom_quat.y = quat[1]
            odom_quat.z = quat[2]
            odom_quat.w = quat[3]

            odom.pose.pose.position = point
            odom.pose.pose.orientation = odom_quat

            odom.child_frame_id = "base_footprint"
            linear = Vector3()
            linear.x = vx
            linear.y = vy
            linear.z = float(0)

            angular = Vector3()
            angular.x = float(0)
            angular.y = float(0)
            angular.z = vth

            odom.twist.twist.linear = linear
            odom.twist.twist.angular = angular

            # Create transform
            odom_tf= TransformStamped()
            odom_tf.transform.translation.x = odom.pose.pose.position.x
            odom_tf.transform.translation.y = odom.pose.pose.position.y
            odom_tf.transform.translation.z = odom.pose.pose.position.z
            odom_tf.transform.rotation = odom.pose.pose.orientation

            odom_tf.header.frame_id = odom.header.frame_id
            odom_tf.child_frame_id = odom.child_frame_id
            odom_tf.header.stamp = odom.header.stamp


            publisher.publish(odom)
            self.tf_broadcaster.sendTransform(odom_tf)


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


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    print(argv)
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    odometry_node = KmpOdometryNode(args.connection,args.robot)

    rclpy.spin(odometry_node)

    #while rclpy.ok():
    #    rclpy.spin_once(odometry_node)
    try:
        odometry_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
