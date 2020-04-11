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


class Test(Node):
    def __init__(self,connection_type,ip):
        super().__init__('test')
        #kmp_odometry_node = rclpy.create_node('test')
        #hh = kmp_odometry_node.get_parameter('connection')
        #print(hh)
        self.get_logger().info('HEI')
        print(TCPSocket)
        #tt = Test2(1234)



def main(argv=sys.argv[1:]):

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-i', '--ip')
    print(argv)
    args = parser.parse_args(remove_ros_args(args=argv))
    rclpy.init(args=argv)
    node = Test(args.connection,args.ip)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()