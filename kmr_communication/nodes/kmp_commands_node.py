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

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse

from script.tcpSocket import TCPSocket
from script.udpSocket import UDPSocket



def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class KmpCommandsNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('kmp_commands_node')
        self.name = 'kmp_commands_node'
        self.declare_parameter('port')
        port = int(self.get_parameter('port').value)
        if robot == 'KMR1':
            self.declare_parameter('KMR1/ip')
            ip = str(self.get_parameter('KMR1/ip').value)
        elif robot == 'KMR2':
            self.declare_parameter('KMR2/ip')
            ip = str(self.get_parameter('KMR2/ip').value)
        else:
            ip=None

        if connection_type == 'TCP':
            self.soc = TCPSocket(ip,port,self.name)
        elif connection_type == 'UDP':
            self.soc=UDPSocket(ip,port,self.name)
        else:
            self.soc=None

        # Make a listener for relevant topics
        sub_twist = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, qos_profile_sensor_data)
        sub_pose = self.create_subscription(Pose, 'pose', self.pose_callback, qos_profile_sensor_data)
        sub_shutdown = self.create_subscription(String, 'shutdown', self.shutdown_callback, qos_profile_sensor_data)

        while not self.soc.isconnected:
            pass
        self.get_logger().info('Node is ready')

        #Dette er brukt for odom og laserscan, tror ikke det er nodvendig ettersom subscriberne har callback, men lar de staa i tilfelle.

        #thread.start_new_thread(self.run, ())

    #def run(self):
    #    while rclpy.ok() and self.soc.isconnected:
    #        self.odom_callback(self.pub_odometry, self.soc.odometry)

    def shutdown_callback(self, data):
        print(data)
        msg = "shutdown"
        self.soc.send(msg)
        #self.udp_soc.isconnected = False

    def twist_callback(self, data):
        msg = 'setTwist ' + str(data.linear.x) + " " + str(data.linear.y) + " " + str(data.angular.z)
        self.soc.send(msg)

    def pose_callback(self, data):
        msg = 'setPose ' + str(data.position.x) + " " + str(data.position.y) + " " + str(data.orientation.z)
        self.soc.send(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    kmp_commands_node = KmpCommandsNode(args.connection,args.robot)

    rclpy.spin(kmp_commands_node)

    #while rclpy.ok():
    #    rclpy.spin_once(odometry_node)
    try:
        kmp_commands_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
