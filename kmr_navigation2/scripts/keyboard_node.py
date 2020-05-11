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
from std_msgs.msg import Bool, String
import sys, select, termios, tty
import rclpy
import _thread as thread
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

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

stopBindings={
        's':(0,0,0),
}


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.settings = termios.tcgetattr(sys.stdin)


        self.warning_field_clear = True
        self.protection_field_clear = True
        self.bool=True

        #Speed given as max_vel_x, max_vel_y, max_vel_theta, max_vel_xy
        #self.highspeed = [0.5, 0.28, 0.5, 0.28]
        #self.lowspeed = [0.1, 0.1, 0.2, 0.1]
        #self.speed = np.array([0.5, 0.28, 0.5, 0.28])
        #self.scaling_factor=1
        self.last_update_time = 0

        self.speed = 0.5 
        self.turn = 0.1 
        x = 0
        y = 0
        z = 0
        th = 0
        self.status = 0


        sub_status = self.create_subscription(KmpStatusdata, 'kmp_statusdata', self.status_callback, qos_profile_sensor_data)
        sub_status = self.create_subscription(Bool, 'clear', self.status2_callback, qos_profile_sensor_data)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stop_pub = self.create_publisher(String, '/shutdown', 10)

        thread.start_new_thread(self.keyboard, ())

    def keyboard(self):
        try:
            print(msg)
            print(self.vels(self.speed,self.turn))
            while(1):
                key = self.getKey()
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                    z = moveBindings[key][2]
                    th = moveBindings[key][3]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    if self.speed > 1.0:
                        self.speed=1.0
                    if self.turn > 1.0:
                        self.turn=1.0
                    print(self.vels(self.speed,self.turn))

                else:
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                    if (key == '\x03'):
                        break

                if (self.status == 14):
                    print(msg)
                self.status = (self.status + 1) % 15


                if key in stopBindings.keys():
                    st = String()
                    st.data="shutdown"
                    print(st)
                    self.stop_pub.publish(st)
                else:
                    twist = Twist()
                    twist.linear.x = x*self.speed
                    twist.linear.y = y*self.speed
                    twist.linear.z = z*self.speed
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = th*self.turn
                    self.pub.publish(twist)
                    print(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)



    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def vels(self,speed,turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)



    def status_callback(self,data):
        if (data.warning_field_clear != self.warning_field_clear and self.get_clock().now().seconds_nanoseconds()[0]-self.last_update_time > 3.0):
            if (data.warning_field_clear == True):
                self.speed=0.5
            if (data.warning_field_clear == False):
                self.speed=0.1
            self.warning_field_clear = data.warning_field_clear
            print(self.warning_field_clear)
            self.last_update_time = self.get_clock().now().seconds_nanoseconds()[0]

    def status2_callback(self,data):
        print(data.data)
        if (data.data != self.bool):
            if (data.data == True):
                 self.speed=0.5
            if (data.data == False):
                self.speed=0.1
            self.bool = data.data


def main(argv=None):
    rclpy.init(args=argv)
    keyboard_node = KeyboardNode()

    rclpy.spin(keyboard_node)
    try:
        keyboard_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
