#!/usr/bin/env python3

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import StaticTransformBroadcaster


class StaticFramePublisher(Node):
    """
    Static broadcast transforms
    This example publishes transforms from `base_link` to a platform and sensor
    frames.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):


        self._tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self._tf_publisher.sendTransform(self.make_transforms())

    def make_transforms(self):
        base_link_to_platform = TransformStamped()
        base_link_to_platform.header.frame_id = 'base_link'
        base_link_to_platform.header.stamp = self.get_clock().now().to_msg()
        base_link_to_platform.child_frame_id = 'platform'
        base_link_to_platform.transform.translation.x = 0.0
        base_link_to_platform.transform.translation.y = 0.0
        base_link_to_platform.transform.translation.z = 0.1
        base_link_to_platform.transform.rotation.w = 1.0
        base_link_to_platform.transform.rotation.x = 0.0
        base_link_to_platform.transform.rotation.y = 0.0
        base_link_to_platform.transform.rotation.z = 0.0

        platform_to_laser_B1 = TransformStamped()
        platform_to_laser_B1.header.frame_id = 'platform'
        platform_to_laser_B1.header.stamp = self.get_clock().now().to_msg()
        platform_to_laser_B1.child_frame_id = 'camera'
        platform_to_laser_B1.transform.translation.x = 0.05
        platform_to_laser_B1.transform.translation.y = 0.0
        platform_to_laser_B1.transform.translation.z = 0.01
        platform_to_laser_B1.transform.rotation.w = 1.0
        platform_to_laser_B1.transform.rotation.x = 0.0
        platform_to_laser_B1.transform.rotation.y = 0.0
        platform_to_laser_B1.transform.rotation.z = 0.0

        platform_to_laser_B4 = TransformStamped()
        platform_to_laser_B4.header.frame_id = 'platform'
        platform_to_laser_B4.header.stamp = self.get_clock().now().to_msg()
        platform_to_laser_B4.child_frame_id = 'sonar_0'
        platform_to_laser_B4.transform.translation.x = 0.05 * math.sin(math.radians(45))
        platform_to_laser_B4.transform.translation.y = 0.05 * math.cos(math.radians(45))
        platform_to_laser_B4.transform.translation.z = 0.0
        platform_to_laser_B4.transform.rotation.w = -0.8733046400935156
        platform_to_laser_B4.transform.rotation.x = 0.0
        platform_to_laser_B4.transform.rotation.y = 0.0
        platform_to_laser_B4.transform.rotation.z = -0.4871745124605095

        # bare her for gøy
        platform_to_sonar_1 = TransformStamped()
        platform_to_sonar_1.header.frame_id = 'platform'
        platform_to_sonar_1.header.stamp = self.get_clock().now().to_msg()
        platform_to_sonar_1.child_frame_id = 'sonar_1'
        platform_to_sonar_1.transform.translation.x = 0.05
        platform_to_sonar_1.transform.translation.y = 0.0
        platform_to_sonar_1.transform.translation.z = 0.0
        platform_to_sonar_1.transform.rotation.w = 1.0
        platform_to_sonar_1.transform.rotation.x = 0.0
        platform_to_sonar_1.transform.rotation.y = 0.0
        platform_to_sonar_1.transform.rotation.z = 0.0

        platform_to_sonar_2 = TransformStamped()
        platform_to_sonar_2.header.frame_id = 'platform'
        platform_to_sonar_2.header.stamp = self.get_clock().now().to_msg()
        platform_to_sonar_2.child_frame_id = 'sonar_2'
        platform_to_sonar_2.transform.translation.x = 0.05 * math.sin(math.radians(45))
        platform_to_sonar_2.transform.translation.y = -0.05 * math.cos(math.radians(45))
        platform_to_sonar_2.transform.translation.z = 0.0
        platform_to_sonar_2.transform.rotation.w = 0.8733046400935156
        platform_to_sonar_2.transform.rotation.x = 0.0
        platform_to_sonar_2.transform.rotation.y = 0.0
        platform_to_sonar_2.transform.rotation.z = -0.4871745124605095

        return (
            base_link_to_platform,
            platform_to_laser_B1,
            platform_to_laser_B4,
        )


def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()