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
import numpy as np

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, PoseWithCovarianceStamped, PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from kmr_msgs.msg import KmpStatusdata

def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class NavigationSupportNode(Node):
    def __init__(self):
        super().__init__('navigation_support_node')


        self.warning_field_clear = True
        self.protection_field_clear = True
        self.bool=True
        self.bool=True

        #Speed given as max_vel_x, max_vel_y, max_vel_theta, max_vel_xy
        self.highspeed = [0.4, 0.4, 0.5, 0.4]
        self.lowspeed = [0.1, 0.1, 0.1, 0.1]
        self.last_update_time = 0

        self.client = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.request = SetParameters.Request()
        while not self.client.wait_for_service(timeout_sec=10.0):
           self.get_logger(        ).info('Waiting for service')

        sub_status = self.create_subscription(KmpStatusdata, 'kmp_statusdata', self.status_callback, qos_profile_sensor_data)
        sub_status = self.create_subscription(Bool, 'clear', self.status2_callback, qos_profile_sensor_data)


        print("OK")



    def setParameter(self,value,name):
        pv = ParameterValue()
        pv.type = 3
        pv.double_value = value
        p = Parameter()
        p.name = name
        p.value = pv
        return p

    def send_velocity_request(self,speed):
        p1 = self.setParameter(speed[0],'max_vel_x')
        p2 = self.setParameter(speed[1],'max_vel_y')
        p3 = self.setParameter(speed[2], 'max_vel_theta')
        p4 = self.setParameter(speed[3], 'max_speed_xy')
        self.request.parameters = [p1,p2,p3,p4]
        wait = self.client.call_async(self.request)
        #rclpy.spin_once(self)
        if wait.result() is not None:
            print(wait.result())
            #self.get_logger().info('Request was ' + self.request.parameters[0].name + '. Response is ' + str(wait.result().results.successful)) # + wait.result().results[1])
        else:
            self.get_logger().info("Request failed")


    def status_callback(self,data):
        if (data.warning_field_clear != self.warning_field_clear):
            if (data.warning_field_clear == True and self.get_clock().now().seconds_nanoseconds()[0]-self.last_update_time > 3.0):
                self.send_velocity_request(self.highspeed)
                self.warning_field_clear = data.warning_field_clear
                print(self.warning_field_clear)
                self.last_update_time = self.get_clock().now().seconds_nanoseconds()[0]
            if (data.warning_field_clear == False):
                self.send_velocity_request(self.lowspeed)
                self.warning_field_clear = data.warning_field_clear
                print(self.warning_field_clear)
                self.last_update_time = self.get_clock().now().seconds_nanoseconds()[0]
            


def main(argv=None):

    rclpy.init(args=argv)
    navigation_support_node = NavigationSupportNode()

    rclpy.spin(navigation_support_node)
    try:
        navigation_support_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
