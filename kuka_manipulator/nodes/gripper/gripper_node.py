#!/usr/bin/env python3

import _thread as thread
import time
import os
import sys
import binascii
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, GoalResponse
from kuka_manipulator.action import OpenGripper
from kuka_manipulator.action import CloseGripper

def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        self.name='gripper_node'
        # TODO: change port to NUC
        #self.ser = serial.Serial(port="/dev/ttyUSB1", baudrate=115200, timeout=1, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

        # Make Publishers for relevant data
        #self.pub_laserscan1 = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)

        self.open_action_server = ActionServer(self,OpenGripper,'open_gripper',self.execute_callback)
        self.close_action_server = ActionServer(self, CloseGripper, 'close_gripper', self.close_gripper_callback)

        self.executing = False

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        while (self.executing):
            pass

        goal_handle.succeed()
        result = OpenGripper.Result()
        result.success = True

        #result.success = self.getOpenResponse()
        #if self.self.getOpenResponse() == True:
        #    goal_handle.succeed()
        #else:
        #    goal_handle.canceled()
        print("OK")
        return result

    def close_gripper_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        while (self.executing):
            pass
        result = CloseGripper.Result()
        result.success = self.getClosedResponse()
        if self.getClosedResponse() == True:
            goal_handle.succeed()
        else:
            goal_handle.canceled()
        return result


    def activate(self):
        #Activation Request
        self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
        data = binascii.hexlify(self.ser.readline()).decode()

        #Status Request until activation complere
        Activated=False
        CompleteString = "09030200005985"
        while(not Activated):
            self.ser.write(b"\x09\x03\x07\xD0\x00\x01\x85\xCF")
            data = binascii.hexlify(self.ser.readline()).decode()
            if(data==CompleteString):
                Activated=True


    def getClosedResponse(self):
        data_raw = self.ser.readline()
        data = binascii.hexlify(data_raw).decode()
        # gripperstatus - null - fault - pos request echo - position - current/force - <null>

        #closing Not complete: 09 03 06 39 00 00 FF 0E 0A F7 8B
        #closing Complete 09 03 06 B9 00 00 FF BD 00 1D 7C
    def getOpenResponse(self):
        t=0
        #Request 09 03 07 D0 00 03 04 0 E
        # opening incomplete 09 03 06 39 00 00 00 BB 10 30 E0
        # opening complete : 09 03 06 F9 00 00 00 0D 00 56 4C


    def close(self):
        # Close the gripper
        # actionrequest - null - null - position - speed - force - <null>
        # to siste FF skal halveres
        self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")

    def open(self):
        # Open the gripper full speed and force
        self.ser.writeser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")

def main(args=None):
    rclpy.init(args=args)
    gripper_node = GripperNode()

    rclpy.spin(gripper_node)

    #while rclpy.ok():
    #    rclpy.spin_once(odometry_node)
    try:
        gripper_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")



if __name__ == '__main__':
    main()
