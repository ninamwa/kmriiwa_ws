#!/usr/bin/env python3

import _thread as thread
import threading
import time
import os
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time


class dummy_odom(Node):
    def __init__(self):
        super().__init__('dummy_odom')

        pub_odometry = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        tf_broadcaster = TransformBroadcaster(self)

        num_readings = 100
        laser_frequency = 40
        a = 0.0
        st=0


        while (True):
            time.sleep(0.1)

            x = 0.0
            y = 0.0
            th = a
            vx = 0.0
            vy = 0.0
            vth = a

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = getTimestamp(self.get_clock().now().nanoseconds)

            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = float(0)

            odom_quat = Quaternion()
            quat = euler_to_quaternion(0, 0, th)
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
            odom_tf = TransformStamped()
            odom_tf.transform.translation.x = odom.pose.pose.position.x
            odom_tf.transform.translation.y = odom.pose.pose.position.y
            odom_tf.transform.translation.z = odom.pose.pose.position.z
            odom_tf.transform.rotation = odom.pose.pose.orientation

            odom_tf.header.frame_id = odom.header.frame_id
            odom_tf.child_frame_id = odom.child_frame_id
            odom_tf.header.stamp = odom.header.stamp

            pub_odometry.publish(odom)
            tf_broadcaster.sendTransform(odom_tf)
            a = a + 0.01

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(
        pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(
        pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(
        pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(
        pitch / 2) * math.sin(yaw / 2)

    return [qx, qy, qz, qw]

def getTimestamp(nano):
        t = nano*10**-9
        timestamp = Time()
        timestamp.sec=math.floor(t)
        timestamp.nanosec=int((t-timestamp.sec)*10**9)
        return timestamp




def main(args=None):
    rclpy.init(args=args)
    node = dummy_odom()
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
