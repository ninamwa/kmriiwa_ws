#!/usr/bin/env python3
import rclpy
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from std_msgs.msg import String

msg = """
Reading from the keyboard!
---------------------------
Moving joints in positive direction:
1 2 3 4 5 6 7
For moving in negative direction, hold down the shift key:
---------------------------
! " # ¤ % & /

Press "0" for stopping the movement in all joints
Press "9" for moving manipulator to Drive Position
Press "s" for shutdown. 

anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
        '1':("a1",1),
        '2':("a2",1),
        '3':("a3",1),
        '4':("a4",1),
        '5':("a5",1),
        '6':("a6",1),
        '7':("a7",1),
        '!':("a1",-1),
        '"':("a2",-1),
        '#':("a3",-1),
        '¤':("a4",-1),
        '%':("a5",-1),
        '&':("a6",-1),
        '/':("a7",-1),
        '0':("a0",0),
        '9':("a9",0)
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

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(key)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def check_if_valid(joint_value):
    if joint_value>1:
        joint_value=1
    elif joint_value<-1:
        joint_value=-1
    return joint_value

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('manipulator_keyboard')
    pub = node.create_publisher(String, '/manipulator_vel', 10)
    stop_pub = node.create_publisher(String, '/shutdown', 10)

    speed = 0.1 #node.get_parameter('speed', 0.5)
    turn = 0.1 #node.get_parameter('turn', 1.0)

    joint = ""
    a1_direction = 0
    a2_direction = 0
    a3_direction = 0
    a4_direction = 0
    a5_direction = 0
    a6_direction = 0
    a7_direction = 0
    status=0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                string = String()
                joint = moveBindings[key][0]
                if joint == "a1":
                    a1_direction = a1_direction + moveBindings[key][1]
                    a1_direction = check_if_valid(a1_direction)
                    string.data = joint + " " + str(a1_direction)
                elif joint == "a2":
                    a2_direction = a2_direction+moveBindings[key][1]
                    a2_direction = check_if_valid(a2_direction)
                    string.data = joint + " " + str(a2_direction)
                elif joint == "a3":
                    a3_direction = a3_direction+moveBindings[key][1]
                    a3_direction = check_if_valid(a3_direction)
                    string.data = joint + " " + str(a3_direction)
                elif joint == "a4":
                    a4_direction = a4_direction+ moveBindings[key][1]
                    a4_direction = check_if_valid(a4_direction)
                    string.data = joint + " " + str(a4_direction)
                elif joint == "a5":
                    a5_direction = a5_direction+moveBindings[key][1]
                    a5_direction = check_if_valid(a5_direction)
                    string.data = joint + " " + str(a5_direction)
                elif joint == "a6":
                    a6_direction = a6_direction+moveBindings[key][1]
                    a6_direction = check_if_valid(a6_direction)
                    string.data = joint + " " + str(a6_direction)
                elif joint == "a7":
                    a7_direction = a7_direction+moveBindings[key][1]
                    a7_direction = check_if_valid(a7_direction)
                    string.data = joint + " " + str(a7_direction)
                elif joint == "a0":
                    string.data = joint + " " + str(moveBindings[key][1])
                    a1_direction = 0
                    a2_direction = 0
                    a3_direction = 0
                    a4_direction = 0
                    a5_direction = 0
                    a6_direction = 0
                    a7_direction = 0
                elif joint == "a9":
                    string.data = joint + " " + str(moveBindings[key][1])
                    a1_direction = 0
                    a2_direction = 0
                    a3_direction = 0
                    a4_direction = 0
                    a5_direction = 0
                    a6_direction = 0
                    a7_direction = 0

                pub.publish(string)
                joint_msg = "Joint1: " + str(a1_direction) + '\n' + "Joint2: " + str(a2_direction) + '\n' + "Joint3: " + str(a3_direction) + '\n' + "Joint4: " + str(a4_direction) + '\n' + "Joint5: " + str(a5_direction) + '\n' + "Joint6: " + str(a6_direction) + '\n' + "Joint7: " + str(a7_direction)
                print(joint_msg)

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                if speed > 1.0:
                    speed=1.0
                if turn > 1.0:
                    turn=1.0

            elif key in stopBindings.keys():
                st = String()
                st.data="shutdown"
                print(st)
                stop_pub.publish(st)

            else:
                if (key == '\x03'):
                    break

            if (status == 14):
                print(msg)
            status = (status + 1) % 15

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
