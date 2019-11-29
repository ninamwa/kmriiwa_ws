#!/usr/bin/env python3
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

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = x+moveBindings[key][0]
                y = y+moveBindings[key][1]
                th = th+moveBindings[key][2]
            elif key in stopBindings.keys():
                st = String()
                st.data="shutdown"
                print(st)
                stop_pub.publish(st)
                break
            else:
                x = x+0.0
                y = y+0.0
                th = th+0.0
                if (key == '\x03'):
                    break

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