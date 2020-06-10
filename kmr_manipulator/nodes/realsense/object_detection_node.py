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
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from kmr_msgs.action import ObjectSearch
from pipeline_srv_msgs.srv import PipelineSrv
from pipeline_srv_msgs.msg import PipelineRequest
from object_analytics_msgs.msg import ObjectsInBoxes3D
from object_analytics_msgs.msg import ObjectInBox3D
from object_msgs.msg import ObjectInBox
from object_msgs.msg import ObjectsInBoxes
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped



def cl_red(msge): return '\033[31m' + msge + '\033[0m'

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.name='object_detection_node'
        self.detection_threshold = 0.5
        self.detected_object_pose = None
        self.isSearching = False

        self.pipelinename = "object"
        self.callback_group = ReentrantCallbackGroup() 

        self.object_detection_action_server = ActionServer(self,ObjectSearch,'object_search',self.object_search_callback, callback_group=self.callback_group)

        self.client = self.create_client(PipelineSrv, '/openvino_toolkit/pipeline_service')
        self.request = PipelineSrv.Request()


        while not self.client.wait_for_service(timeout_sec=10.0):
           self.get_logger().info('Waiting for service')

        sub_LocalizedObject = self.create_subscription(ObjectsInBoxes3D, '/object_analytics/localization', self.detectedObject_callback, qos_profile_sensor_data, callback_group=self.callback_group)

        self.last_instance = None


    def detectedObject_callback(self, ObjectsInBoxes):
        probability=0
        for instance in ObjectsInBoxes.objects_in_boxes:
            probability = instance.object.probability
            self.last_instance= instance
        if(self.last_instance != None and self.isSearching):
            if(self.last_instance.object.probability>=self.detection_threshold):
                self.detected_object_pose = self.getBoundingBoxMidPoint(self.last_instance.min, self.last_instance.max)
                print("OBJECT DETECTED")
                self.endSearch()

    def object_search_callback(self, goal_handle):
        self.startSearch()
        starttime = time.time()
        self.get_logger().info('Executing goal...')
        elapsed = 0
        while (self.isSearching and elapsed<20):
            elapsed = time.time() - starttime
            pass
        print("done searching")
        if self.isSearching:
           self.endSearch()
        result = ObjectSearch.Result()
        if self.detected_object_pose != None:
            result.success = True
            result.pose = self.detected_object_pose
            self.detected_object_pose = None
        if result.success == True:
            goal_handle.succeed()
        else:
            print("aborting goal")
            goal_handle.abort()
        return result

    def startSearch(self):
        self.isSearching = True
        #self.send_pipeline_request("RUN_PIPELINE")


    def endSearch(self):
        self.isSearching = False
        #self.send_pipeline_request("PAUSE_PIPELINE")


    def getBoundingBoxMidPoint(self,min,max):
        midpoint = PoseStamped()
        midpoint.pose.position.x = min.x - (min.x-max.x)/2
        midpoint.pose.position.y = min.y - (min.y-max.y)/2
        midpoint.pose.position.z = min.z - (min.z-max.z)/2
        midpoint.pose.orientation.x = 0.0 
        midpoint.pose.orientation.y = 0.0 
        midpoint.pose.orientation.z = 0.7372773  
        midpoint.pose.orientation.w = 0.6755902  
        midpoint.header.frame_id = "camera_color_optical_frame"
        return midpoint


    def send_pipeline_request(self,msg):
        self.request.pipeline_request.cmd = msg
        self.request.pipeline_request.value = self.pipelinename
        wait = self.client.call_async(self.request)
        if wait.result() is not None:
            print("Wait result" + wait.result())
            self.get_logger().info('Request was ' + self.request.parameters[0].name + '. Response is ' + str(wait.result().results.successful)) # + wait.result().results[1])
        else:
            self.get_logger().info("Request sent")

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(object_detection_node,executor)
    try:
        object_detection_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")



if __name__ == '__main__':
    main()
