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
import os
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PoseStamped
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from kuka_manipulator.action import OpenGripper, CloseGripper, ObjectSearch, DriveToFrame
from kuka_communication.msg import LbrStatusdata
from functools import partial


def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class ManipulatorNode(Node):
    def __init__(self):
        super().__init__('manipulator_node')
        self.name='manipulator_node'

      
        self.pub_moveit_pose = self.create_publisher(PoseStamped, '/moveit/goalpose', qos_profile_sensor_data)

        sub_path_finished = self.create_subscription(Bool,'/path_finished',self.path_finished_callback, qos_profile_sensor_data)
        sub_navgoal_reached= self.create_subscription(Bool,'/navgoal_reached',self.navgoal_callback, qos_profile_sensor_data)
        self.opengripper_action_client = ActionClient(self, OpenGripper, 'open_gripper')
        self.closegripper_action_client = ActionClient(self, CloseGripper, 'close_gripper')
        self.camera_search_action_client = ActionClient(self, ObjectSearch, 'camera_object_search')
        self.drivetoframe_action_client = ActionClient(self, DriveToFrame, '/moveit/frame')

        self.gripperStatus = "OPEN"
        self.hasObject = False
        self.pathFinished = False

        self.frame1_busy = False
        self.frame2_busy = False
        self.frame3_busy =  False

        self.search_counter = 0
        self.send_new_navgoal = False
    

    #TODO: Hvordan skal det gjøres når noe skal plukkes AV roboten??? 
    #TODO: Håntere hva vi gjør i retry() - bevege robot litt bakover og prøve igjen, how? 
    #TODO: Lage metode for å sende nytt NAV GOAL. 

    def navgoal_callback(self,data):
        if data.data == True:
            self.search_counter = 0
            self.send_drivetoframe_goal("search_1")

    def path_finished_callback(self,data):
        self.pathFinished = data.data

        if self.send_new_navgoal:
            #SEND NY NAV_GOAL
            self.send_new_navgoal = False
        else:
            if self.pathFinished and self.hasObject: 
                self.send_open_gripper_goal()
            elif self.pathFinished and not self.hasObject:
                self.send_close_gripper_goal()
           
    def send_open_gripper_goal(self):
        print("Open gripper goal")
        goal_msg = OpenGripper.Goal()
        self.opengripper_action_client.wait_for_server()
        self._send_goal_future = self.opengripper_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(partial(self.goal_response_callback, type="open"))

    def send_close_gripper_goal(self):
        print("Close gripper goal")
        goal_msg = CloseGripper.Goal()
        self.closegripper_action_client.wait_for_server()
        self._send_goal_future = self.closegripper_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(partial(self.goal_response_callback, type="close"))

    def goal_response_callback(self, future, type):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(partial(self.get_result_callback,type=type))

    def get_result_callback(self, future, type):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        if type == "open" and result.success:
            self.gripperStatus = "OPEN"
            self.hasObject = False
            self.send_drivetoframe_goal("drive_frame")
        elif type == "close":
            if result.success:
                self.gripperStatus = "CLOSED"
                self.hasObject = True
                self.send_drivetoframe_goal(None)
            else:
                self.gripperStatus = "CLOSED"
                self.hasObject = False
                self.retry()
        elif type == "objectsearch":
            if result.success:
                self.pub_moveit_pose.publish(result.pose)
                self.search_counter = 0
            else:
                if self.search_counter < 3:
                    goal = "search_" + str(self.search_counter+1)
                    self.send_drivetoframe_goal(goal)
                else:
                    self.send_drivetoframe_goal("drive_frame")
        elif type == "drivetoframe"
            if result.success:
                if result.frame == "frame1":
                    self.frame1_busy=True
                elif result.frame == "frame2":
                    self.frame2_busy=True
                elif result.frame == "frame3":
                    self.frame3_busy=True
                elif result.frame == "drive_frame":
                    self.send_new_navgoal = True
            else:
                #Was not possible to create plan - put object down and continue. 
                self.send_open_gripper_goal()
                print(cl_red('Error: ') + "Was not able to plan to desired frame")
        elif type=="drivetoframe_search" 
            self.search_counter += 1
            if result.success:
                self.send_objectsearch_goal()
            else:
                #Failed to move to correct search frame, move on to the next!
                goal = "search_" + str(self.search_counter+1)
                self.send_drivetoframe_goal(goal)

        
    

    def send_objectsearch_goal(self):
        print("Object Search goal")
        goal_msg = ObjectSearch.Goal()
        self.camera_search_action_client.wait_for_server()
        self._send_goal_future = self.camera_search_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(partial(self.goal_response_callback, type="objectsearch"))

    def retry(self):
            #Be Moveit gå litt bakover
            self.send_objectsearch_goal()

    def send_drivetoframe_goal(self,goal):
        print("DriveToFrame goal")
        type = "drivetoframe"
        goal_msg = DriveToFrame.Goal()
        if goal == "drive_frame":
            goal_msg.frame = goal
        elif goal.split("_")[0] == "search":
            goal_msg.frame = goal
            type = type + "_search"
        else:
            if self.frame1_busy == False:
                goal_msg.frame = "frame1"
            elif self.frame2_busy == False:
                goal_msg.frame = "frame2"
            elif self.frame3_busy == False:
                goal_msg.frame = "frame3"
            else:
                #If no frame is empty, the gripper should open and leave the object in the same place
                self.send_open_gripper_goal()
                return
        self.drivetoframe_action_client.wait_for_server()
        self._send_goal_future = self.drivetoframe_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(partial(self.goal_response_callback, type="drivetoframe"))

    def getTimestamp(self,nano):
        t = nano * 10 ** -9
        timestamp = Time()
        timestamp.sec = math.floor(t)
        timestamp.nanosec = int((t - timestamp.sec) * 10 ** 9)
        return timestamp


def main(args=None):
    rclpy.init(args=args)
    manipulator_node = ManipulatorNode()
    rclpy.spin(manipulator_node)

    try:
        manipulator_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")



if __name__ == '__main__':
    main()
