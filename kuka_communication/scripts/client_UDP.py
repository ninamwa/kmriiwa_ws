#!/usr/bin/env python3



#######################################################################################################################
import _thread as thread
import threading
import time
import os
import math
import rclpy
from rclpy.node import Node
import socket
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import qos_profile_sensor_data


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
class UDPSocket:
    #   M: __init__ ===========================
    def __init__(self):
        self.BUFFER_SIZE = 4096
        #self.BUFFER_SIZE = 10000
        self.isconnected = False
        self.isFinished = (False, None)
        self.hasError = (False, None)
        self.isready = False
        self.odometry = []
        self.laserScanB1 = []
        self.laserScanB4 = []
        self.udp = None

        #TODO: Do something with isready, which is relevant for us.
        threading.Thread(target=self.connect_to_socket).start()
        #try:
        #    threading.Thread(target=self.connect_to_socket).start()
        #except:
        #    print(cl_pink("Error: ") + "Unable to start connection thread")

    def close(self):
        self.isconnected = False

    def connect_to_socket(self):
        # TODO: REPLACE THIS WHEN CONFIG.TXT IS FIXED
        ros_host="192.168.10.116"
        ros_port = 30001

        os.system('clear')
        print(cl_pink('\n=========================================='))
        print(cl_pink('<   <  < << INITIALIZE UDPconnection>> >  >   >'))
        print(cl_pink('=========================================='))
        print(cl_pink(' KUKA API for ROS2'))
        print(cl_pink('==========================================\n'))

        print(cl_cyan('Starting up on:'), 'IP:', ros_host, 'Port:', ros_port)
        try:
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.settimeout(0.1)
            self.udp.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)
            self.udp.bind((ros_host, ros_port))
        except:
            print(cl_red('Error: ') + "Connection for KUKA cannot assign requested address:", ros_host, ros_port)
            os._exit(-1)


        print(cl_cyan('Waiting for a connection...'))
        while (not self.isconnected):
            try:
                data, self.client_address = self.udp.recvfrom(self.BUFFER_SIZE)
                self.isconnected = True
            except:
                t=0
        print(cl_cyan('Connection from: '), self.client_address)
        print(cl_cyan('Message: '), data.decode('utf-8'))

        self.udp.sendto("hello KUKA".encode('utf-8'), self.client_address)
        print("Responded KUKA")


        timee = time.time() #For debugging purposes
        count = 0
        while self.isconnected:
            try:
                data, self.client_address = self.udp.recvfrom(self.BUFFER_SIZE)
                data = data.decode('utf-8')
                last_read_time = time.time()  # Keep received time
                # Process the received data package
                cmd_splt=data.split(">")[1].split()
                if len(cmd_splt) and cmd_splt[0] == 'isFinished':
                    if cmd_splt[1] == "false":
                        self.isFinished = False
                    elif cmd_splt[1] == "true":
                        self.isFinished = True
                if len(cmd_splt) and cmd_splt[0] == 'hasError':
                    if cmd_splt[1] == "false":
                        self.hasError = False
                    elif cmd_splt[1] == "true":
                        self.hasError = True
                if len(cmd_splt) and cmd_splt[0] == 'odometry':
                        self.odometry = cmd_splt
                if len(cmd_splt) and cmd_splt[0] == 'laserScan':
                    if cmd_splt[2] == '1801':
                        self.laserScanB1.append(cmd_splt)
                        # print(cmd_splt)
                        count = count + 1
                        print(count)
                    elif cmd_splt[2] == '1802':
                        self.laserScanB4.append(cmd_splt)
                        count = count + 1
                        print(count)

            except:
                t=0
                #elapsed_time = time.time() - last_read_time
                #if elapsed_time > 5.0:  # Didn't receive a pack in 5s
                #  print("exception!!")
                #  self.isconnected = False
                #  print(cl_lightred('No packet received from iiwa for 5s!'))

        print("SHUTTING DOWN")
        self.udp.close()
        self.isconnected = False
        print(cl_lightred('Connection is closed!'))
        rclpy.shutdown()

    # Each send command runs as a thread. May need to control the maximum running time (valid time to send a command).
    def send(self, cmd):
        try:
            thread.start_new_thread(self.__send, (cmd,))
        except:
            print(cl_red('Error: ') + "sending message thread failed")

    def __send(self, cmd):
        encoded_cmd = cmd.encode() # Encode to bytes
        self.udp.sendto(encoded_cmd, self.client_address)


###   Class: Kuka iiwa ROS node    ###
class KukaCommunication:
    def __init__(self,node,connection):
        self.kuka_communication_node = node
        self.udp_soc = connection

        self.last_odom_timestamp = 0
        self.last_scan_timestamp = 0

        # Make a listener for relevant topics
        sub_twist = self.kuka_communication_node.create_subscription(Twist, 'cmd_vel', self.twist_callback, qos_profile_sensor_data)
        sub_pose = self.kuka_communication_node.create_subscription(Pose, 'pose', self.pose_callback, qos_profile_sensor_data)
        sub_shutdown = self.kuka_communication_node.create_subscription(String, 'shutdown', self.shutdown_callback, qos_profile_sensor_data)
        kuka_subscriber = self.kuka_communication_node.create_subscription(String, 'kuka_command', self.callback, qos_profile_sensor_data)
        # TODO: RATE er enda ikke implementert
        # self.rate = self.kuka_node.create_rate(100) # 100 hz

        # Make Publishers for all kuka_iiwa data
        pub_isFinished = self.kuka_communication_node.create_publisher(String, 'isFinished', qos_profile_sensor_data)
        pub_hasError = self.kuka_communication_node.create_publisher(String, 'hasError', qos_profile_sensor_data)
        pub_odometry = self.kuka_communication_node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        pub_laserscan1 = self.kuka_communication_node.create_publisher(LaserScan, 'scan_1', qos_profile_sensor_data)
        pub_laserscan4 = self.kuka_communication_node.create_publisher(LaserScan, 'scan_2', qos_profile_sensor_data)
        self.send_static_transform()

        # Create tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(node)

        while not self.udp_soc.isconnected:
            pass
        print('Ready to start')

        thread.start_new_thread(self.executor, ())

        while rclpy.ok() and self.udp_soc.isconnected:
            #string_callback(pub_isFinished, self.udp_soc.isFinished)
            #string_callback(pub_hasError,self.udp_soc.hasError)
            self.odom_callback(pub_odometry,self.udp_soc.odometry)
            if len(self.udp_soc.laserScanB1):
                self.scan_callback(pub_laserscan1, self.udp_soc.laserScanB1.pop(0))
            if len(self.udp_soc.laserScanB4):
                self.scan_callback(pub_laserscan4, self.udp_soc.laserScanB4.pop(0))
            #TODO: Rate igjen? Charlotte?
            #self.rate.sleep() #100 hz rate.sleep()


    #TODO: Hva skal vi gjøre her? Publishe alt som strings eller lage egendefinerte ROS meldinger?
    def string_callback(self,publisher,values):
        msg = String()
        ros_timestamp = Time()
        ros_timestamp.sec = time.time()
        msg.data= str(values[0]) + " %s" % time.time()
        publisher.publish(msg)


    def odom_callback(self, publisher, values):
        if (len(values) == 8 and values[1] != self.last_odom_timestamp):
            kuka_timestamp = values[1]
            self.last_odom_timestamp = kuka_timestamp

            x = float(values[2].split(":")[1])
            y = float(values[3].split(":")[1])
            th = float(values[4].split(":")[1])
            vx = float(values[5].split(":")[1])
            vy = float(values[6].split(":")[1])
            vth = float(values[7].split(":")[1])


            odom = Odometry()
            odom.header.stamp = self.getTimestamp2(self.kuka_communication_node.get_clock().now().nanoseconds)
            odom.header.frame_id = "odom"

            point = Point()
            point.x = x
            point.y = y
            point.z = float(0)

            odom_quat = Quaternion()
            quat = self.euler_to_quaternion(0,0,th)
            odom_quat.x = quat[0]
            odom_quat.y = quat[1]
            odom_quat.z = quat[2]
            odom_quat.w = quat[3]

            odom.pose.pose.position = point
            odom.pose.pose.orientation = odom_quat

            odom.child_frame_id = "base_footprint"
            linear = Vector3()
            linear.x = vx
            linear.y = vy
            linear.z = float(0)

            angular = Vector3()
            angular.x = float(0)
            angular.y = float(0)
            angular.z = vth

            odom.twist.twist.linear = linear
            odom.twist.twist.angular = angular

            # Create transform
            odom_tf= TransformStamped()
            odom_tf.transform.translation.x = odom.pose.pose.position.x
            odom_tf.transform.translation.y = odom.pose.pose.position.y
            odom_tf.transform.translation.z = odom.pose.pose.position.z
            odom_tf.transform.rotation = odom.pose.pose.orientation

            odom_tf.header.frame_id = odom.header.frame_id
            odom_tf.child_frame_id = odom.child_frame_id
            odom_tf.header.stamp = odom.header.stamp


            publisher.publish(odom)
            self.tf_broadcaster.sendTransform(odom_tf)


    def scan_callback(self, publisher, values):
        if (len(values) == 4 and values[1] != self.last_scan_timestamp):
            kuka_timestamp = values[1]
            self.last_scan_timestamp =kuka_timestamp
            scan = LaserScan()
            #scan.header.stamp = self.getTimestamp(float(kuka_timestamp))
            scan.header.stamp = self.getTimestamp2(self.kuka_communication_node.get_clock().now().nanoseconds)
            if values[2] == '1801':
                scan.header.frame_id = "scan_1"
            elif values[2] == '1802':
                scan.header.frame_id="scan_2"
            scan.angle_increment = (0.5*math.pi)/180
            scan.angle_min = (-135*math.pi)/180
            scan.angle_max = (135*math.pi)/180
            scan.range_min = 0.12 # disse må finnes ut av
            scan.range_max = 3.5 # finn ut
            ranges=values[3].split(",")
            try:
                scan.ranges=[float(i) for i in ranges if len(i.strip())]
                #scan.intensities=[float(10) for i in ranges]
            except ValueError as e:
                print("error", e)

            publisher.publish(scan)


    def callback(self, data):
        self.udp_soc.send(data.data)  # e.g 'setPosition 45 60 0 -25 0 95 0' for going to start position

    def shutdown_callback(self, data):
        print(data)
        msg = 'shutdown'
        self.udp_soc.send(msg)
        #self.udp_soc.isconnected = False


    def twist_callback(self, data):
        msg = 'setTwist ' + str(data.linear.x) + " " + str(data.linear.y) + " " + str(data.angular.z)
        self.udp_soc.send(msg)

    def pose_callback(self, data):
        msg = 'setPose ' + str(data.position.x) + " " + str(data.position.y) + " " + str(data.orientation.z)
        self.udp_soc.send(msg)


    def executor(self):
        while rclpy.ok():
            rclpy.spin_once(self.kuka_communication_node)
        try:
            self.kuka_communication_node.destroy_node()
            rclpy.shutdown()
        except:
            print(cl_red('Error: ') + "rclpy shutdown failed")

    def getTimestamp(self, kuka_timestamp):
        timestamp = Time()
        timestamp.sec=math.floor(kuka_timestamp)
        timestamp.nanosec=int((kuka_timestamp-timestamp.sec)*10**9)
        return timestamp

    def getTimestamp2(self,nano):
        t = nano * 10 ** -9
        timestamp = Time()
        timestamp.sec = math.floor(t)
        timestamp.nanosec = int((t - timestamp.sec) * 10 ** 9)
        return timestamp

    def euler_to_quaternion(self, roll, pitch, yaw):

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]

    def listToString(list):
        listString = str(list.x) + " " + str(list.y) + " " + str(list.z)
        return listString

    def send_static_transform(self):
        broadcaster1 = StaticTransformBroadcaster(self.kuka_communication_node)
        broadcaster2 = StaticTransformBroadcaster(self.kuka_communication_node)
        static_transformStamped = TransformStamped()
        static_transformStamped.header.frame_id = "laser_B4_link"
        static_transformStamped.child_frame_id = "scan_2"
        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0
        quat = self.euler_to_quaternion(0, 0, 0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
        broadcaster1.sendTransform(static_transformStamped)
        static_transformStamped.header.frame_id = "laser_B1_link"
        static_transformStamped.child_frame_id = "scan_1"
        broadcaster2.sendTransform(static_transformStamped)


##################################################
#   M:  Reading config file for Server IP and Port
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
    return [IP, Port]


def main(args=None):
    rclpy.init(args=None)
    kuka_communication_node = rclpy.create_node("kuka_communication_node")
    udp = UDPSocket()
    KukaCommunication(kuka_communication_node,udp)

    #try:
    #    threading.Thread(target=KukaCommunication(kuka_communication_node,udp)).start()
    #except:
    #    print(cl_pink("Error: ") + "Unable to start connection thread")
    #comm = KukaCommunication(kuka_communication_node,udp)


if __name__ == '__main__':
    main()
