#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import sys
from rclpy.utilities import remove_ros_args
import argparse
from TCPSocket import TCPSocket
from nodes.test2 import Test2


class Test(Node):
    def __init__(self,connection_type,ip):
        super().__init__('test')
        #kmp_odometry_node = rclpy.create_node('test')
        #hh = kmp_odometry_node.get_parameter('connection')
        #print(hh)
        self.get_logger().info('HEI')
        print(TCPSocket)
        tt = Test2(1234)


def main(argv=sys.argv[1:]):

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-i', '--ip')
    print(argv)
    args = parser.parse_args(remove_ros_args(args=argv))
    rclpy.init(args=argv)
    node = Test(args.connection,args.ip)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()