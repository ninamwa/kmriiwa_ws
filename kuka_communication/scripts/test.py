#!/usr/bin/env python3

import _thread as thread
import threading
import time
import os
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time


class Kuka:
    def __init__(self):
        rclpy.init(args=None)
        kuka_communication_node = rclpy.create_node("kuka_communication_node")
        pub_odometry = kuka_communication_node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        scan_pub = kuka_communication_node.create_publisher(LaserScan, 'scan_1', qos_profile_sensor_data)
        scan_pub2 = kuka_communication_node.create_publisher(LaserScan, 'scan_2', qos_profile_sensor_data)
        tf_broadcaster = TransformBroadcaster(kuka_communication_node)

        num_readings = 100
        laser_frequency = 40
        ranges = []
        intensities = []
        print("HEI")

        broadcaster1 = StaticTransformBroadcaster(kuka_communication_node)
        broadcaster2 = StaticTransformBroadcaster(kuka_communication_node)
        static_transformStamped = TransformStamped()
        static_transformStamped.header.frame_id = "laser_B4_link"
        static_transformStamped.child_frame_id = "scan_2"
        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0
        quat = euler_to_quaternion(0, 0, 0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
        broadcaster1.sendTransform(static_transformStamped)
        static_transformStamped.header.frame_id = "laser_B1_link"
        static_transformStamped.child_frame_id = "scan_1"
        broadcaster2.sendTransform(static_transformStamped)

        a = 0.0
        while (True):

            # time.sleep(1)

            count = 0
            scan = LaserScan()
            # scan.header.frame_id = "scan"
            scan.angle_min = -1.57
            scan.angle_max = 1.57
            scan.header.stamp = getTimestamp(kuka_communication_node.get_clock().now().nanoseconds)
            scan.header.frame_id = "scan_1"
            scan.angle_increment = 3.14 / num_readings
            scan.time_increment = (1 / laser_frequency) / (num_readings)
            scan.range_min = 0.0
            scan.range_max = 100.0

            for i in range(1, num_readings - 1):
                scan.ranges = [float(10) for i in range(0, num_readings)]
                scan.intensities = [float(100) for i in range(0, num_readings)]
            scan_pub.publish(scan)

            scan = LaserScan()
            # scan.header.frame_id = "scan"
            scan.angle_min = -1.57
            scan.angle_max = 1.57
            scan.header.stamp = getTimestamp(kuka_communication_node.get_clock().now().nanoseconds)
            scan.header.frame_id = "scan_2"
            scan.angle_increment = 3.14 / num_readings
            scan.time_increment = (1 / laser_frequency) / (num_readings)
            scan.range_min = 0.0
            scan.range_max = 100.0

            for i in range(1, num_readings - 1):
                scan.ranges = [float(10) for i in range(0, num_readings)]
                scan.intensities = [float(100) for i in range(0, num_readings)]
            scan_pub2.publish(scan)

            count = count + 1

            x = 0.0
            y = 0.0
            th = a
            vx = 0.0
            vy = 0.0
            vth = a

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = getTimestamp(kuka_communication_node.get_clock().now().nanoseconds)

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

            # TODO: Sjekk om denne egentlig skal være base_footprint
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

            a = a + 1

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
    try:
        threading.Thread(target=Kuka).start()
    except:
        print("Error: Unable to start connection thread")


if __name__ == '__main__':
    main()
