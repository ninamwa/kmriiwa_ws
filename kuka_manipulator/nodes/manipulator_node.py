#!/usr/bin/env python3

import _thread as thread
import time
import os
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from kuka_manipulator.action import OpenGripper, CloseGripper, ObjectSearch
from kuka_communication.msg import LbrStatusdata
from functools import partial


def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class ManipulatorNode(Node):
    def __init__(self):
        super().__init__('manipulator_node')
        self.name='manipulator_node'

      
        self.pub_moveit_pose = self.create_publisher(PoseStamped, '/moveit/goalpose', qos_profile_sensor_data)
        self.pub_moveit_frame = self.create_publisher(String, '/moveit/frame', qos_profile_sensor_data)

        sub_path_finished = self.create_subscription(LbrStatusdata,'/path_finished',self.path_finished_callback, qos_profile_sensor_data)
        self.opengripper_action_client = ActionClient(self, OpenGripper, 'open_gripper')
        self.closegripper_action_client = ActionClient(self, CloseGripper, 'close_gripper')
        self.camera_search_action_client = ActionClient(self, ObjectSearch, 'camera_object_search')

        self.gripperStatus = "OPEN"
        self.hasObject = False
        self.pathFinished = False

        self.frame1 = None
        self.frame2 = None
        self.frame3 =  None
     
        thread.start_new_thread( self.send_close_gripper_goal, ())
        time.sleep(10)
        thread.start_new_thread( self.send_open_gripper_goal, ())
    

    def path_finished_callback(self,data):
        self.pathFinished = data
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
        elif type == "close" and result.success:
            self.gripperStatus = "CLOSED"
            self.hasObject = True
            self.driveToFrame()
        elif type == "close" and not result.success:
            self.gripperStatus = "CLOSED"
            self.hasObject = False
            self.retry()
        elif type == "objectsearch" and result.success:
            self.pub_moveit_pose.publish(result.pose)
        elif type == "objectsearch" and not result.success:
            t=0
            #Håntere hva vi gjør hvis kamera ikke finner object
        
    

    def send_objectsearch_goal(self):
        print("Object Search goal")
        goal_msg = ObjectSearch.Goal()
        self.camera_search_action_client.wait_for_server()
        self._send_goal_future = self.camera_search_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(partial(self.goal_response_callback, type="objectsearch"))

    def retry(self):
            #Be Moveit gå litt bakover
            self.send_objectsearch_goal()

    def driveToFrame(self):
        if self.frame1 == None:
            frame = "frame1"
        elif self.frame2 == None:
            frame = "frame2"
        elif self.frame3 == None:
            frame = "frame3"
        else:
            frame=None
            #Håndtere hvis ingen er tomme
        self.pub_moveit_pose.publish(frame)
        ##Når settes disse til å ikke være NONE ?? 
        


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
