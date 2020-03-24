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


import time
import math
from std_msgs.msg import Bool

import rclpy
import threading
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, PoseWithCovarianceStamped, PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from kuka_communication.msg import KmpStatusdata

def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')


        self.warning_field_clear = True
        self.protection_field_clear = True
        self.bool=True

        #Speed given as max_vel_x, max_vel_y, max_vel_theta, max_vel_xy
        #self.highspeed = [0.5, 0.28, 0.5, 0.28]
        #self.lowspeed = [0.13, 0.1, 0.2, 0.1]
        self.highspeed = [0.26, 0.0, 1.0, 0.26]
        self.lowspeed = [0.1, 0.0, 0.5, 0.1]
        self.highestspeed = [0.5, 0.0, 1.5, 0.5]
        self.last_update_time = 0

        #self.action_client = ActionClient(self, NavigateToPose, '/NavigateToPose')
        #self.waypoint_client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')
        #while not self.action_client.wait_for_server(timeout_sec=10.0):
        #    self.get_logger().info('Waiting for NavigateToPose service')
        #while not self.waypoint_client.wait_for_server(timeout_sec=10.0):
        #    self.get_logger().info('Waiting for Waypoint service')

        self.client = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.request = SetParameters.Request()
        while not self.client.wait_for_service(timeout_sec=10.0):
           self.get_logger(        ).info('Waiting for service')

        initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', qos_profile_sensor_data)

        sub_status = self.create_subscription(KmpStatusdata, 'kmp_statusdata', self.status_callback, qos_profile_sensor_data)
        sub_status = self.create_subscription(Bool, 'clear', self.status2_callback, qos_profile_sensor_data)

        st=0
        while(False):
            #time.sleep(0.1)
            if (st == 25):
                initial = PoseWithCovarianceStamped()
                initial.header.frame_id = "map"
                initial.header.stamp = self.getTimestamp(self.get_clock().now().nanoseconds)

                point = Point()
                point.x = -2.0
                point.y = 0.0
                point.z = 0.0

                quat = Quaternion()
                quat.x = 0.0
                quat.y = 0.0
                quat.z = 0.0
                quat.w = 0.1

                initial.pose.pose.position = point
                initial.pose.pose.orientation = quat

                #initial_pub.publish(initial)

            if (st == 80):
                t=0
                #self.send_goal()
                #self.send_waypoint_goal()

            #print(st)
            st=st+1

    def create_pose(self,x,y,th):
        a = PoseStamped()
        a.header.frame_id = "map"
        a.header.stamp = self.getTimestamp(self.get_clock().now().nanoseconds)

        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0

        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = 0.0
        quat.w = th

        a.pose.position = point
        a.pose.orientation = quat
        return a

    def send_waypoint_goal(self):
        goal = FollowWaypoints.Goal()
        goal.poses = []
        a = self.create_pose(1.81, -0.57,1.0)
        goal.poses.append(a)
        b = self.create_pose(0.64, 1.83, 1.0)
        goal.poses.append(b)
        c = self.create_pose(-2.0, 0.0, 1.0)
        goal.poses.append(c)


        print(str(goal.poses))
        self.waypoint_client.send_goal_async(goal, feedback_callback=self.feedback_callback)


    def send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.getTimestamp(self.get_clock().now().nanoseconds)

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


    def setParameter(self,value,name):
        pv = ParameterValue()
        pv.type = 3
        pv.double_value = value
        p = Parameter()
        p.name = name
        p.value = pv
        return p

    def send_velocity_request(self,speed):
        p1 = self.setParameter(speed[0],'FollowPath.max_vel_x')
        p2 = self.setParameter(speed[1],'FollowPath.max_vel_y')
        p3 = self.setParameter(speed[2], 'FollowPath.max_vel_theta')
        p4 = self.setParameter(speed[3], 'FollowPath.max_speed_xy')
        self.request.parameters = [p1,p2,p3,p4]
        wait = self.client.call_async(self.request)
        #rclpy.spin_once(self)
        if wait.result() is not None:
            print(wait.result())
            #self.get_logger().info('Request was ' + self.request.parameters[0].name + '. Response is ' + str(wait.result().results.successful)) # + wait.result().results[1])
        else:
            self.get_logger().info("Request failed")


    def status_callback(self,data):
        if (data.warning_field_clear != self.warning_field_clear and self.get_clock().now().seconds_nanoseconds()[0]-self.last_update_time > 5.0):
            if (data.warning_field_clear == True):
                self.send_velocity_request(self.highspeed)
            if (data.warning_field_clear == False):
                #threading.Thread(target=self.send_velocity_request).start()
                self.send_velocity_request(self.lowspeed)
            self.warning_field_clear = data.warning_field_clear
            print(self.warning_field_clear)
            self.last_update_time = self.get_clock().now().seconds_nanoseconds()[0]

    def status2_callback(self,data):
        print(data.data)
        if (data.data != self.bool):
            if (data.data == True):
                self.send_velocity_request(self.highspeed)
            if (data.data == False):
                self.send_velocity_request(self.lowspeed)
            self.bool = data.data



    def getTimestamp(self,nano):
        t = nano * 10 ** -9
        timestamp = Time()
        timestamp.sec = math.floor(t)
        timestamp.nanosec = int((t - timestamp.sec) * 10 ** 9)
        return timestamp


def main(argv=None):

    rclpy.init(args=argv)
    navigation_node = NavigationNode()

    rclpy.spin(navigation_node)

    #while rclpy.ok():
    #    rclpy.spin_once(odometry_node)
    try:
        navigation_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
