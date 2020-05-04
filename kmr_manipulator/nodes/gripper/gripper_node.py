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

import _thread as thread
import time
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
from gripper_msgs import GripperMsg
from errorcodes import ErrorCodes
from kmr_msgs.action import Gripper


from enum import Enum
def cl_red(msge): return '\033[31m' + msge + '\033[0m'


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        self.name='gripper_node'
        self.ser = serial.Serial(port="/dev/ttyUSB1", baudrate=115200, timeout=1, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        self.activate()
        print("Activated") 
        self.gripper_action_server = ActionServer(self,Gripper,'move_gripper',self.move_gripper_callback)

    def move_gripper_callback(self, goal_handle):
        self.get_logger().info('Executing gripper goal...')
        result = Gripper.Result()
        if goal_handle.request.action == "open":
            self.open()
            while (self.isMoving()):
                pass
            result.success = self.getOpenResponse()
        elif goal_handle.request.action == "close":
            self.close()
            while (self.isMoving()):
                pass
            result.success = self.getClosedResponse()

        if result.success == True:
           goal_handle.succeed()
        else:
           goal_handle.abort()
        print("GRIPPER ACTION OK")
        return result


    def activate(self):
        # Activation Request
        self.ser.write(GripperMsg.ClearMemory.value)
        data_raw = self.ser.readline()
        self.ser.write(GripperMsg.ActivationRequest.value)
        data_raw = self.ser.readline()
        activated = False
        while (not activated):
            self.ser.write(GripperMsg.ActivationStatusRequest.value)
            response = self.ser.readline()
            gSTA=self.response_to_gSTA(response)
            if (gSTA == GripperMsg.ACTIVATIONCOMPLETE.value):
                activated = True
            if (gSTA == GripperMsg.ACTIVATIONINPROGRESS.value):
                print("Gripper activation in progress...")

    def close(self):
        # Close the gripper
        self.ser.write(GripperMsg.CloseRequest.value)
        self.ser.readline()

    def open(self):
        # Open the gripper full speed and force
        self.ser.write(GripperMsg.OpenRequest.value)
        self.ser.readline()

    def isMoving(self):
        result=False
        self.ser.write(GripperMsg.MotionStatusRequest.value)
        gOBJ = self.response_to_gOBJ(str(self.ser.readline()))
        # Not Moving
        if(gOBJ==GripperMsg.NOTMOVING.value):
            result=False
            print("Not moving")
        # Moving
        if(gOBJ==GripperMsg.MOVING.value):
            result=True
            print("Moving")
        return result

    def getClosedResponse(self):
        result = False
        self.ser.write(GripperMsg.MotionStatusRequest.value)
        gOBJ = self.response_to_gOBJ(str(self.ser.readline()))
        # Closed, no object:
        if(gOBJ==GripperMsg.REQUESTEDPOSITION.value):
            result=False
            print(ErrorCodes.NO_OBJECT)
            # Closed, object:
        if(gOBJ==GripperMsg.OBJECT_CLOSING.value):
            result=True
            print(ErrorCodes.OBJECT_FOUND)
        return result

    def getOpenResponse(self):
        result=False
        self.ser.write(GripperMsg.MotionStatusRequest.value)
        gOBJ = self.response_to_gOBJ(str(self.ser.readline()))
        # Collision
        if (gOBJ == GripperMsg.OBJECT_OPENING.value):
            result = False
            print(ErrorCodes.COLLISION)
            # Open
        if (gOBJ == GripperMsg.REQUESTEDPOSITION.value):
            result = True
            print(ErrorCodes.OPEN)
        return result

    def response_to_gSTA(self,data_string):
        binascii = str(binascii.hexlify(response))
        gSTA = list(binascii)[6] # eller 8 hvis b og ' kommer med
        return int(gSTA)

    def response_to_gOBJ(self,data_string):
        # moving only meaningful if gGTO = 1
        hexa = data_string.split("\\")[4].split("x")[1]
        binary = bin(int(hexa, 16))[2:].zfill(8)
        value = 2*int(binary[0]) + 1*int(binary[1])
        gGTO = binary[4]
        if (value==0 and gGTO!=1):
            value=-1
        return value


def main(args=None):
    rclpy.init(args=args)
    gripper_node = GripperNode()

    rclpy.spin(gripper_node)
    try:
        gripper_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")



if __name__ == '__main__':
    main()
