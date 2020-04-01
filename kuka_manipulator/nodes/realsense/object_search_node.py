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
import time
import sys
import serial
import binascii
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, GoalResponse
from kuka_manipulator.action import ObjectSearch

def cl_red(msge): return '\033[31m' + msge + '\033[0m'


class ObjectSearchNode(Node):
    def __init__(self):
        super().__init__('object_search_node')
        self.name='object_search_node'
        # TODO: change port to NUC
        self.object_search_action_server = ActionServer(self,ObjectSearch,'object_search',self.object_search_callback)    


    def object_search_callback(self, goal_handle):
        self.startSearch()
        self.get_logger().info('Executing goal...')
        while (self.isSearching()):
            pass
        result = ObjectSearch.Result()
        if self.getObject() != None:
            result.success = True
            result.pose = self.getObject()
        if result.success == True:
           goal_handle.succeed()
        else:
           goal_handle.abort()
        return result

    def startSearch(self):
        #start søket på en eller annen måte?
        t=0

    def isSearching(self):
        result=False
        #Et eller annet som returnerer hvorvidt kamera er "ferdig" eller ei, slik at vi vet når vi kan gå videre og retrieve resultatet 
        return result

    def getObject(self):
        pose = None
        #posen må være av følgende type: geometry_msgs/PoseStamped pose
        #Returner posen på objectet
        return pose


def main(args=None):
    rclpy.init(args=args)
    object_search_node = ObjectSearchNode()

    rclpy.spin(object_search_node)
    try:
        object_search_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")



if __name__ == '__main__':
    main()
