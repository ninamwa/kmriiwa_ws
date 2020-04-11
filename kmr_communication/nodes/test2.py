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

#from script.test import Test
#from script.tcpSocket import *
from scripts import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data


class Test2(Node):
    def __init__(self):
        super().__init__('lbr_commands_node')
        sub_pathplanning = self.create_subscription(JointTrajectory, '/fake_joint_trajectory_controller/joint_trajectory', self.path_callback,qos_profile_sensor_data)

    def path_callback(self, data):
        i=1
        for point in data.points:
            positions = " ".join([str(s) for s in point.positions])
            velocities = " ".join([str(s) for s in point.velocities])
            accelerations = " ".join([str(s) for s in point.accelerations])
            if i == 1:
                type = "StartPoint"
            elif i ==len(data.points):
                type = "EndPoint"
            else:
                type = "WayPoint"
            msg = 'pathPointLBR ' + ">" + type + ">" + positions + ">" + velocities + ">" + accelerations
            print(msg)
            i+=1

def main(argv=sys.argv[1:]):
    rclpy.init(args=argv)
    lbr_commands_node = Test2()

    rclpy.spin(lbr_commands_node)

    lbr_commands_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
