#!/usr/bin/env python3

import _thread as thread
import time
import sys
import math
import rclpy
from rclpy.node import Node
from kuka_communication.msg import KmpStatusdata
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from script.tcpSocket import TCPSocket
from script.udpSocket import UDPSocket

from rclpy.utilities import remove_ros_args
import argparse

def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class KmpStatusNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('kmp_statusdata_node')
        self.name='kmp_statusdata_node'
        self.last_status_timestamp = 0
        self.declare_parameter('port')
        port = int(self.get_parameter('port').value)
        if robot == 'KMR1':
            self.declare_parameter('KMR1/ip')
            ip = str(self.get_parameter('KMR1/ip').value)
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

        # Make Publisher for statusdata
        self.pub_kmp_statusdata = self.create_publisher(KmpStatusdata, 'kmp_statusdata', qos_profile_sensor_data)


        while not self.soc.isconnected:
            pass
        self.get_logger().info('Node is ready')

        thread.start_new_thread(self.run, ())

    def run(self):
        while rclpy.ok() and self.soc.isconnected:
            self.status_callback(self.pub_kmp_statusdata, self.soc.kmp_statusdata)

    def status_callback(self,publisher,data):
        # TODO: Fyll inn med riktig statusdata - dette maa ogsaa gjores i selve meldingsfila - den er naa random.
        if data != None:
            msg = KmpStatusdata()
            msg.header.stamp = self.getTimestamp(self.get_clock().now().nanoseconds)
            status_elements = data[1].split(",")
            if (status_elements[1] != self.last_status_timestamp):
                self.last_status_timestamp = status_elements[1]
                for i in range(2, len(status_elements)):
                    split = status_elements[i].split(":")
                    if(split[0]=="OperationMode"):
                        msg.operation_mode = split[1]
                    elif (split[0] == "ReadyToMove"):
                        if (split[1] == "true"):
                            msg.ready_to_move = True
                        else:
                            msg.ready_to_move = False
                    elif (split[0] == "WarningField"):
                        if (split[1] == "true"):
                            msg.warning_field_clear = True
                        else:
                            msg.warning_field_clear = False
                    elif (split[0] == "ProtectionField"):
                        if (split[1] == "true"):
                            msg.protection_field_clear = True
                        else:
                            msg.protection_field_clear = False
                    elif (split[0] == "isKMPmoving"):
                        if (split[1] == "true"):
                            msg.is_kmp_moving = True
                        else:
                            msg.is_kmp_moving = False
                    elif (split[0] == "KMPsafetyStop"):
                        if (split[1] == "true"):
                            msg.kmp_safetystop = True
                        else:
                            msg.kmp_safetystop = False
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
    kmp_statusdata_node = KmpStatusNode(args.connection,args.robot)

    rclpy.spin(kmp_statusdata_node)

    #while rclpy.ok():
    #    rclpy.spin_once(odometry_node)
    try:
        kmp_statusdata_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
