#!/usr/bin/env python3

import _thread as thread
import time
import os
import sys
import serial
import binascii
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, GoalResponse
from kuka_manipulator.action import OpenGripper
from kuka_manipulator.action import CloseGripper

from enum import Enum
def cl_red(msge): return '\033[31m' + msge + '\033[0m'


class Msg(Enum):
    Activation = b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30"
    ActivationRequest = b"\x09\x03\x07\xD0\x00\x01\x85\xCF"
    ActivationComplete = "09030200005985"

    StatusResponse= b"\x09\x03\x07\xD0\x00\x03\x04\x0E"
    CloseRequest = b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\x7F\x7F\x22\x49"
    OpenRequest = b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19"

    NOTMOVING = -1
    MOVING = 0
    OBJECT_OPENING = 1
    OBJECT_CLOSING = 2
    REQUESTEDPOSITION = 3


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
        # Activation Request
        self.ser.write(Msg.Activation.value)
        data_raw = self.ser.readline()
        Activated = False
        while (not Activated):
            self.ser.write(Msg.ActivationRequest.value)
            data_raw = self.ser.readline()
            data_string = binascii.hexlify(data_raw).decode()
            if (data_string == Msg.ActivationComplete.value):
                Activated = True
                self.close()
                self.open()

    def close(self):
        # Close the gripper
        self.ser.write(Msg.CloseRequest.value)
        self.ser.readline()

    def open(self):
        # Open the gripper full speed and force
        self.ser.write(Msg.OpenRequest.value)
        self.ser.readline()

    def getClosedResponse(self):
        result = False
        while (not result):
            self.ser.write(Msg.StatusResponse.value)
            data_raw = self.ser.readline()
            print(data_raw)
            gObj = self.messageTogObj(str(data_raw))
            # Not Moving
            if(gObj==Msg.NOTMOVING.value):
                result=False
                print("Not moving")
            # Moving
            if(gObj==Msg.MOVING.value):
                result=False
                print("Closing")
            # Closed, no object:
            if(gObj==Msg.REQUESTEDPOSITION.value):
                result=False
                print("No object found while closing")
            # Closed, object:
            if(gObj==Msg.OBJECT_CLOSING.value):
                result=True
                print("Object found while closing")
        return result

    def getOpenResponse(self):
        result=False
        while(not result):
            self.ser.write(Msg.StatusResponse.value)
            gObj = self.messageTogObj(str(self.ser.readline()))
            if(gObj==Msg.NOTMOVING.value):
                result=False
                print("Not moving")
            #Moving
            if (gObj == Msg.MOVING.value):
                result = False
                print("Opening")
            # Collision
            if (gObj == Msg.OBJECT_OPENING.value):
                result = False
                print("Collision")
            # Open
            if (gObj == Msg.REQUESTEDPOSITION.value):
                result = True
                print("Open: Requested Position")
        return result


    def messageTogObj(self, data_string):
        # moving only meaningful if gGTO = 1
        hexa = data_string.split("\\")[4].split("x")[1]
        print(hexa)
        binary = bin(int(hexa, 16))[2:].zfill(8)
        gGTO = binary[4]
        gObj = 2*int(binary[0]) + 1*int(binary[1])
        if (gObj==0 and gGTO!=1):
            gObj=-1
        return gObj

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
