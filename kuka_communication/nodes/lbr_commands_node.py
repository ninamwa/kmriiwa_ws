#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from script.tcpSocket import TCPSocket
from script.udpSocket import UDPSocket



def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class LbrCommandsNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('lbr_commands_node')
        self.name = 'lbr_commands_node'
        self.declare_parameter('port')
        port = int(self.get_parameter('port').value)
        if robot == 'KMR1':
            self.declare_parameter('/KMR1/ip')
            ip = str(self.get_parameter('/KMR1/ip').value)
        elif robot == 'KMR2':
            self.declare_parameter('KMR2/ip')
            ip = str(self.get_parameter('KMR2/ip').value)
        else:
            ip=None

        if connection_type == 'TCP':
            self.soc = TCPSocket(ip,port,self.name)
        elif connection_type == 'UDP':
            self.soc=UDPSocket(ip,port,self.name)
        else:
            self.soc=None

        # Make a listener for relevant topics
        sub_manipulator_vel = self.create_subscription(String, 'manipulator_vel', self.manipulatorVel_callback, qos_profile_sensor_data)
        sub_shutdown = self.create_subscription(String, 'shutdown', self.shutdown_callback, qos_profile_sensor_data)
        sub_pathplanning = self.create_subscription(JointTrajectory, '/fake_joint_trajectory_controller/joint_trajectory', self.path_callback, qos_profile_sensor_data)

        while not self.soc.isconnected:
            pass
        self.get_logger().info('Node is ready')


    def shutdown_callback(self, data):
        print(data)
        msg = "shutdown"
        self.soc.send(msg)
        #self.udp_soc.isconnected = False

    def manipulatorVel_callback(self, data):
        msg = 'setLBRmotion ' + data.data
        self.soc.send(msg)

    def path_callback(self, data):
        i=1
        for point in data.points:
            positions = " ".join([str(s) for s in point.positions])
            velocities = " ".join([str(s) for s in point.velocities])
            accelerations = " ".join([str(s) for s in point.accelerations])
            if i == 1:
                type = "StartPoint"
            elif i ==len(data.points):
                type = "EndPoint"
            else:
                type = "WayPoint"
            msg = 'pathPointLBR ' + ">" + type + ">" + positions + ">" + velocities + ">" + accelerations
            self.soc.send(msg)
            i+=1


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    lbr_commands_node = LbrCommandsNode(args.connection,args.robot)

    rclpy.spin(lbr_commands_node)

    #while rclpy.ok():
    #    rclpy.spin_once(odometry_node)
    try:
        lbr_commands_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
