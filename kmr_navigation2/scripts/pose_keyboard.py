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
import sys, select, termios, tty
from geometry_msgs.msg import Pose
from std_msgs.msg import String

msg = """
Reading from the keyboard  and Publishing to Pose!
---------------------------
Moving around:
   w  e  r
   a  s  d
      c  
"""

moveBindings = {
        'w':(0.0,0.0,100.0),
        'e':(100.0,0.0,0.0),
        'r': (0.0, 0.0, -100.0),
        'a':(0.0,100.0,0.0),
        'd':(0.0,-100.0,0.0),
        'c': (-100.0, 0.0, 0.0),
    }
stopBindings={
        's':(0,0,0),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('pose_keyboard')
    pub = node.create_publisher(Pose, '/pose', 10)
    stop_pub = node.create_publisher(String, '/shutdown', 10)

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status=0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
            elif key in stopBindings.keys():
                st = String()
                st.data="shutdown"
                print(st)
                stop_pub.publish(st)
            else:
                x = 0.0
                y = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if (status == 14):
                print(msg)
            status = (status + 1) % 15

            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = 0.0
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = th
            print(p)
            pub.publish(p)

    except Exception as e:
        print(e)

    finally:
        t = Pose()
        t.position.x = x+0.0
        t.position.y = y+0.0
        t.position.z = 0.0
        t.orientation.x = 0.0
        t.orientation.y = 0.0
        t.orientation.z = th+0.0
        print(t)
        pub.publish(t)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)