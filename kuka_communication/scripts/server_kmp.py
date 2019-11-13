#!/usr/bin/env python3

# KUKA API for ROS

version = '26092019'

# Marhc 2017 Saeid Mokaram  saeid.mokaram@gmail.com
# Sheffield Robotics    http://www.sheffieldrobotics.ac.uk/
# The university of sheffield   http://www.sheffield.ac.uk/

# This script generats a ROS node for comunicating with KUKA iiwa
# Dependencies: conf.txt, ROS server, Rospy, KUKA iiwa java SDK, KUKA iiwa robot.

#######################################################################################################################
import _thread as thread
import time
import os
import select
import sys
import termios
import tty
import rclpy
import socket
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from time import sleep


def cl_black(msge): return '\033[30m' + msge + '\033[0m'
def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'
def cl_orange(msge): return '\033[33m' + msge + '\033[0m'
def cl_blue(msge): return '\033[34m' + msge + '\033[0m'
def cl_purple(msge): return '\033[35m' + msge + '\033[0m'
def cl_cyan(msge): return '\033[36m' + msge + '\033[0m'
def cl_lightgrey(msge): return '\033[37m' + msge + '\033[0m'
def cl_darkgrey(msge): return '\033[90m' + msge + '\033[0m'
def cl_lightred(msge): return '\033[91m' + msge + '\033[0m'
def cl_lightgreen(msge): return '\033[92m' + msge + '\033[0m'
def cl_yellow(msge): return '\033[93m' + msge + '\033[0m'
def cl_lightblue(msge): return '\033[94m' + msge + '\033[0m'
def cl_pink(msge): return '\033[95m' + msge + '\033[0m'
def cl_lightcyan(msge): return '\033[96m' + msge + '\033[0m'


#######################################################################################################################
#   Class: Kuka iiwa TCP communication    #####################
class iiwa_socket:
    #   M: __init__ ===========================
    def __init__(self, ip, port):
        self.BUFFER_SIZE = 1024
        self.isconnected = False
        self.isFinished = (False, None)
        self.hasError = (False, None)
        self.isready = False
        self.odometry = ([None,None,None,None,None,None])
        self.laserScan = (None,None)
        self.UDPsocket = None


        #TODO: Do something with isready, which is relevant for us.

        try:
            # Starting connection thread
            thread.start_new_thread(self.socket, (ip, port,))
        except:
            print(cl_pink("Error: ") + "Unable to start connection thread")

    def close(self):
        self.isconnected = False

    def socket(self, ip, port):
        self.UDPsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind the socket to the port
        server_address = (ip, port)

        # TODO: REPLACE THIS WHEN CONFIG.TXT IS FIXED
        local_hostname = socket.gethostname()
        local_ip= socket.gethostbyname(local_hostname)
        local_port = 23456
        server_address = (local_ip, local_port)

        os.system('clear')
        print(cl_pink('\n=========================================='))
        print(cl_pink('<   <  < << INITIALIZE UDPconnection>> >  >   >'))
        print(cl_pink('=========================================='))
        print(cl_pink(' KUKA API for ROS2'))
        print(cl_pink('==========================================\n'))

        print(cl_cyan('Starting up on:'), 'IP:', ip, 'Port:', port)
        try:
            self.UDPsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.UDPsocket.bind(server_address)
        except:
            print(cl_red('Error: ') + "Connection for KUKA cannot assign requested address:", ip, port)
            os._exit(-1)

            # Wait for a connection
            print(cl_cyan('Waiting for a connection...'))

        data, self.client_address = self.UDPsocket.recvfrom(self.BUFFER_SIZE)
        print(cl_cyan('Connection from: '), self.client_address)
        print(cl_cyan('Message: '), data)

        self.UDPsocket.sendto("hello KUKA".encode(), self.client_address)
        print("Responded KUKA")

        self.isconnected = True

        last_read_time = time.time()
        while self.isconnected:
            try:
                data, addr = self.UDPsocket.recvfrom(self.BUFFER_SIZE)
                ######  TEST DECODE #######
                # data.decode("utf-8")
                data = data.decode()
                ##########################
                last_read_time = time.time()  # Keep received time
                # Process the received data package
                for pack in data.split(">"):  # parsing data pack
                    cmd_splt = pack.split()

                    ##REMOVE last_read_time from messages, not used for anything!
                    if len(pack) and cmd_splt[0] == 'isFinished':  # If isFinished
                        if cmd_splt[1] == "false":
                            self.isFinished = (False, last_read_time)
                        elif cmd_splt[1] == "true":
                            self.isFinished = (True, last_read_time)
                    if len(pack) and cmd_splt[0] == 'hasError':  # If hasError
                        if cmd_splt[1] == "false":
                            self.hasError = (False, last_read_time)
                        elif cmd_splt[1] == "true":
                            self.hasError = (True, last_read_time)
                    if len(pack) and cmd_splt[0] == 'odometry':
                            tmp = [float(''.join([c for c in s if c in '0123456789.eE-'])) for s in cmd_splt[1:]]
                            if len(tmp) == 6:
                                self.odometry = (tmp, last_read_time)
                    if len(pack) and cmd_splt[0] == 'laserScan':
                            tmp = [float(''.join([c for c in s if c in '0123456789.eE-'])) for s in cmd_splt[1:]]
                            if len(tmp) == 6:
                                self.odometry = (tmp, last_read_time)

            except:
                elapsed_time = time.time() - last_read_time
                if elapsed_time > 5.0:  # Didn't receive a pack in 5s
                    print("exception!!")
                    self.close()  # Disconnect from iiwa
                    self.isconnected = False
                    print(cl_lightred('No packet received from iiwa for 5s!'))

        print("SHUTTING DOWN")
        self.connection.shutdown(self.socket.SHUT_RDWR)
        self.connection.close()
        self.UDPsocket.close()
        self.isconnected = False
        print(cl_lightred('Connection is closed!'))
        # NOE JEG SLANG Paa SELV :)
        rclpy.shutdown()

    # Each send command runs as a thread. May need to control the maximum running time (valid time to send a command).
    def send(self, cmd):
        thread.start_new_thread(self.__send, (cmd,))

    def __send(self, cmd):
        ## Add lines between commands
        # cmd = cmd + '\r\n'
        ## Encode to bytes
        encoded_cmd = cmd.encode()

        ## Send commands
        self.UDPsocket.sendto(encoded_cmd, self.client_address)


#   ~Class: Kuka iiwa TCP communication    #####################
#######################################################################################################################

#######################################################################################################################
#   Class: Kuka iiwa ROS node    #####################
class kuka_iiwa_ros2_node:
    #   M: __init__ ===========================
    def __init__(self, ip, port):  # Makes kuka_iiwa ROS2 node
        self.iiwa_soc = iiwa_socket(ip, port)
        while (not self.iiwa_soc.isready):
            pass
        print("Ready to go!")
        #    Make a listener for relevant topics
        rclpy.init(args=None)
        self.kuka_node = rclpy.create_node("kuka_iiwa")
        kuka_twist_subscriber = self.kuka_node.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        kuka_pose_subscriber = self.kuka_node.create_subscription(Pose, 'cmd_vel', self.pose_callback, 10)
        self.rate = self.kuka_node.create_rate(100) # 100 hz
        kuka_subscriber = self.kuka_node.create_subscription(String, 'kuka_command', self.callback, 10)


        #   Make Publishers for all kuka_iiwa data
        pub_isFinished = self.kuka_node.create_publisher(String, 'isFinished', 10)
        pub_hasError = self.kuka_node.create_publisher(String, 'hasError', 10)
        pub_odometry = self.kuka_node.create_publisher(Odometry, 'odom',10)
        pub_laserscan = self.kuka_node.create_publisher(LaserScan, 'scan', 10)
        thread.start_new_thread(self.executor, ())

        # while not rospy.is_shutdown() and self.iiwa_soc.isconnected:
        while rclpy.ok() and self.iiwa_soc.isconnected:
            print()
            #string_callback(pub_isFinished, self.iiwa_soc.isFinished)
            #string_callback(pub_hasError,self.iiwa_soc.hasError)
            #odom_callback(pub_odometry, self.iiwa_soc.odometry)
            #scan_callback(pub_laserscan, self.iiwa_soc.laserScan)
            self.rate.sleep() #100 hz rate.sleep()


        # while not rospy.is_shutdown() and self.iiwa_soc.isconnected:
        while rclpy.ok() and self.iiwa_soc.isconnected:
            # data_str = self.iiwa_soc.data + " %s" % rospy.get_time()
            #   Update all the kuka_iiwa data
            for [pub, values] in [[pub_isFinished, self.iiwa_soc.isFinished],
                                [pub_hasError, self.iiwa_soc.hasError]]:
                msg = String()
                msg.data= str(values[0]) + " %s" % time.time()
                # Denne tiden maa dobbeltsjekkes at blir riktig!
                # I ros 1 er det  data_str = str(data[0]) +' '+ str(rospy.get_time())

                #For å poste til terminal
                # kuka_node.get_logger().info('Publishing: "%s"' % msg.data)

                pub.publish(msg)

            self.rate.sleep() #100 hz rate.sleep()

    def string_callback(self,publisher,values):
        msg = String()
        msg.data= str(values[0]) + " %s" % time.time()
        publisher.publish(msg)

    def odom_callback(self, publisher, values):
        x = values[0]
        y = values[1]
        th = values[2]
        vx = values[3]
        vy = values[4]
        vth = values[5]

        odom = Odometry()
        odom.header.stamp = self.kuka_node.get_clock().now()
        odom.header.frame_id = "odom"
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
        odom.pose.pose = Pose(Point(x, y, 0), Quaternion(odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        publisher.publish(odom)

    def scan_callback(self, publisher, values):
        # TODO: M_PI må settes :) "how to make a laserscan message"
        M_PI = 0
        scan = LaserScan()
        scan.header.stamp = self.kuka_node.get_clock().now()
        scan.angle_increment = (2.0*M_PI/360.0)
        scan.angle_min = 0.0
        scan.angle_max = 2.0*M_PI-scan
        scan.range_min = 0.12
        scan.range_max = 3.5
        #scan.ranges.resize(360)
        #scan.intensities.resize(360)
        scan.ranges = values
        publisher.publish(scan)



    #   M: callback ===========================
    #   Receiving command string and sending it to KUKA iiwa
    def callback(self, data):
        ###########rospy.loginfo(rospy.get_caller_id() + "Received command " + str(data.data) )
        self.iiwa_soc.send(data.data)  # e.g 'setPosition 45 60 0 -25 0 95 0' for going to start position


    def twist_callback(self, data):
        msg = 'setTwist ' + listToString(data.linear) + " " + listToString(data.angular)
        self.iiwa_soc.send(msg)

    def pose_callback(self, data):
        msg = 'setPose ' + listToString(data.position.x) + " " + listToString(data.position.y) + " " + listToString(data.quaternion.z)
        self.iiwa_soc.send(msg)


    def executor(self):
        while rclpy.ok():
            rclpy.spin_once(self.kuka_node)
        self.kuka_node.destroy_node()
        rclpy.shutdown()




#   ~Class: Kuka iiwa ROS2 node    #####################
######################################################################################################################

#   M:  Reading config file for Server IP and Port =================
def read_conf():
    f_conf = os.path.abspath(os.path.dirname(__file__)) + '/config.txt'
    if os.path.isfile(f_conf):
        IP = ''
        Port = ''
        for line in open(f_conf, 'r'):
            l_splt = line.split()
            if len(l_splt) == 4 and l_splt[0] == 'server':
                IP = l_splt[1]
                Port = int(l_splt[3])
        if len(IP.split('.')) != 4 or Port <= 0:
            print(cl_red('Error:'), "conf.txt doesn't include correct IP/Port! e.g. server 172.31.1.50 port 1234")
            exit()
    else:
        print(cl_red('Error:'), "conf.txt doesn't exist!")
        #exit()
    return [IP, Port]

def listToString(list):
    strl = " "
    return strl.join([str(elem) for elem in list])
#   ~M:  Reading config file for Server IP and Port ================

def main(args=None):
    #rclpy.init(args=args)
    [IP, Port] = read_conf()
    try:
        node = kuka_iiwa_ros2_node(IP, Port)  # Make a Kuka_iiwa ROS node
    except Exception:
        print("An error occured, the system has crashed!")
        # rclpy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
