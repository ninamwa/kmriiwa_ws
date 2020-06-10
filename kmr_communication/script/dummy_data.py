#!/usr/bin/env python3

# Copyright 2019 Nina Marie Wahl and Charlotte Heggem.
# Copyright 2019 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import _thread as thread
import threading
import time
import os
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class Kuka(Node):
    def __init__(self):
        super().__init__('kmr_communication_node')

        #self.client = self.create_client(SetParameters, '/controller_server/set_parameters')
        #self.request = SetParameters.Request()
        #while not self.client.wait_for_service(timeout_sec=10.0):
        #    self.get_logger(        ).info('Waiting for service')

        ip = '192.168.10.117'
        port = 30001
        #self.soc = TCPSocket(ip, port)
        #self.soc = UDPSocket(ip, port)


        #Type of action, action name
        #self.action_client = ActionClient(self, NavigateToPose, '/NavigateToPose')
        #while not self.action_client.wait_for_server(timeout_sec=10.0):
        #    self.get_logger(        ).info('Waiting for service')



        pub_odometry = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        scan_pub = self.create_publisher(LaserScan, 'scan_1', qos_profile_sensor_data)
        scan_pub2 = self.create_publisher(LaserScan, 'scan_2', qos_profile_sensor_data)
        tf_broadcaster = TransformBroadcaster(self)

        initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', qos_profile_sensor_data)
        goal_pub = self.create_publisher(PoseStamped, 'move_base_simple/goal', qos_profile_sensor_data)

        num_readings = 100
        laser_frequency = 40
        ranges = []#from script.test import Test
#from script.tcpSocket import *
        intensities = []

        broadcaster1 = StaticTransformBroadcaster(self)
        broadcaster2 = StaticTransformBroadcaster(self)
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
        st=0


        while (True):
            time.sleep(0.1)

            count = 0
            scan = LaserScan()
            # scan.header.frame_id = "scan"
            scan.angle_min = -1.57
            scan.angle_max = 1.57
            scan.header.stamp = getTimestamp(self.get_clock().now().nanoseconds)
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
            scan.header.stamp = getTimestamp(self.get_clock().now().nanoseconds)
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

            #if (st==300):
            #    self.send_request(0.6)

            #if (st == 600):
            #    self.send_request(0.7)

            a = a + 1

            if (st==25):
                initial = PoseWithCovarianceStamped()

                initial.header.frame_id = "map"
                initial.header.stamp = getTimestamp(self.get_clock().now().nanoseconds)

                point = Point()
                point.x = 0.1
                point.y = 0.0
                point.z = 0.0

                quat = Quaternion()
                quat.x = 0.0
                quat.y = 0.0
                quat.z = 0.0
                quat.w = 0.1

                initial.pose.pose.position = point
                initial.pose.pose.orientation = quat


                initial_pub.publish(initial)

            if (st==50):
                self.send_goal()

            #print(st)
            #st = st +1
        time.sleep(3)
        print("hei")
        count=0

    def send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = getTimestamp(self.get_clock().now().nanoseconds)

        point = Point()
        point.x = -2.44
        point.y = -1.29
        point.z = 0.0

        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = 0.0
        quat.w = 1.0

        goal.pose.pose.position = point
        goal.pose.pose.orientation = quat

        self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(feedback)
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

    def send_request(self,value):

        pv = ParameterValue()
        pv.type = 3
        pv.double_value = value
        p = Parameter()
        p.name = 'max_vel_x'
        p.value = pv
        self.request.parameters = [p]
        wait = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, wait)
        if wait.result() is not None:
            print(wait.result())
            #self.get_logger().info('Request was ' + self.request.parameters[0].name + '. Response is ' + str(wait.result().results.successful)) # + wait.result().results[1])
        else:
            self.get_logger().info("Request failed")


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
    node = Kuka()
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
