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

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import sys
from rclpy.utilities import remove_ros_args
import argparse
from tcpSocket import TCPSocket
from nodes.test2 import Test2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data


class Test(Node):
    def __init__(self):
        super().__init__('test')
       
        self.callback_group = ReentrantCallbackGroup()
        sub_1=self.create_subscription(String, 'topic1', self.callback1, qos_profile_sensor_data,callback_group=self.callback_group)
        sub_2=self.create_subscription(String, 'topic2', self.callback2, qos_profile_sensor_data,callback_group=self.callback_group)

        self.isMoving = False

    
    def callback1(self, data):
        self.get_logger().info('callback1')
        self.isMoving = True
        while (self.isMoving):
            print("ismoving")
            pass
        self.get_logger().info('UTE AV CALLBACK 1')
        
    
    def callback2(self,data):
        self.get_logger().info('callback2')
        if (data.data=="OK"):
            self.isMoving=False
            print("OK")
        
        

def main(argv=None):
    rclpy.init(args=argv)
    node = Test()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()