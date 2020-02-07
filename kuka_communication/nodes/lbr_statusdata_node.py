#!/usr/bin/env python3

import _thread as thread
import sys
import math
import rclpy
from rclpy.node import Node
from kuka_communication.msg import LbrStatusdata
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from script.tcpSocket import TCPSocket
from script.udpSocket import UDPSocket

from rclpy.utilities import remove_ros_args
import argparse

def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class LbrStatusNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('lbr_statusdata_node')
        self.name='lbr_statusdata_node'
        self.declare_parameter('port')
        port = int(self.get_parameter('port').value)
        if robot == 'KMR1':
            self.declare_parameter('/KMR1/ip')
            ip = str(self.get_parameter('/KMR1/ip').value)
        elif robot == 'KMR2':
            self.declare_parameter('KMR2/ip')
            ip = str(self.get_parameter('KMR2/ip').value)
        else:
            ip = None


        if connection_type == 'TCP':
            self.soc = TCPSocket(ip,port,self.name)
        elif connection_type == 'UDP':
            self.soc=UDPSocket(ip,port,self.name)
        else:
            self.soc=None

        self.last_odom_timestamp = 0

        # Make Publisher for statusdata
        self.pub_lbr_statusdata = self.create_publisher(LbrStatusdata, 'lbr_statusdata', qos_profile_sensor_data)


        while not self.soc.isconnected:
            pass
        self.get_logger().info('Node is ready')

        thread.start_new_thread(self.run, ())

    def run(self):
        while rclpy.ok() and self.soc.isconnected:
            self.status_callback(self.pub_lbr_statusdata, self.soc.lbr_statusdata)



    def status_callback(self,publisher,values):
        if values != None:
            msg = LbrStatusdata()
            msg.header.stamp = self.getTimestamp(self.get_clock().now().nanoseconds)
            msg.header.frame_id = "baselink"


            #TODO: Fyll inn med riktig statusdata - dette maa ogsaa gjores i selve meldingsfila - den er naa random.
            #msg.data= str(values[0])

            publisher.publish(msg)


    def getTimestamp(self,nano):
        t = nano * 10 ** -9
        timestamp = Time()
        timestamp.sec = math.floor(t)
        timestamp.nanosec = int((t - timestamp.sec) * 10 ** 9)
        return timestamp


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    lbr_statusdata_node = LbrStatusNode(args.connection,args.robot)

    rclpy.spin(lbr_statusdata_node)

    #while rclpy.ok():
    #    rclpy.spin_once(odometry_node)
    try:
        lbr_statusdata_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()