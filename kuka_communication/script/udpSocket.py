#!/usr/bin/env python3
import _thread as thread
import threading
import time
import os
import rclpy
import socket




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


class UDPSocket:
    def __init__(self,ip,port,node):
        self.BUFFER_SIZE = 4096
        self.isconnected = False
        self.node_name = node
        self.ip = ip
        self.port = port
        self.udp = None

        #Data
        self.odometry = []
        self.laserScanB1 = []
        self.laserScanB4 = []
        self.lbr_sensordata = []
        self.kmp_statusdata = None
        self.lbr_statusdata = None


        #TODO: Do something with isready, which is relevant for us.
        threading.Thread(target=self.connect_to_socket).start()
        #try:
        #    threading.Thread(target=self.connect_to_socket).start()
        #except:
        #    print(cl_pink("Error: ") + "Unable to start connection thread")

    def close(self):
        self.isconnected = False

    def connect_to_socket(self):

        #print(cl_cyan('Starting up on:'), 'IP:', ros_host, 'Port:', ros_port)
        print(cl_cyan('Starting up node:'), self.node_name, 'IP:', self.ip, 'Port:', self.port)
        try:
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.settimeout(0.1)
            self.udp.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)
            self.udp.bind((self.ip, self.port))
        except:
            print(cl_red('Error: ') + "Connection for KUKA cannot assign requested address/node:",self.node_name, self.ip, self.port)
            os._exit(-1)


        #print(cl_cyan('Waiting for a connection...'))
        while (not self.isconnected):
            try:
                data, self.client_address = self.udp.recvfrom(self.BUFFER_SIZE)
                self.isconnected = True
            except:
                t=0
        #print(cl_cyan('Connection from: '), self.client_address)
        #print(cl_cyan('Message: '), data.decode('utf-8'))

        self.udp.sendto("hello KUKA".encode('utf-8'), self.client_address)
        #print("Responded KUKA")


        timee = time.time() #For debugging purposes
        count = 0
        while self.isconnected:
            try:
                data, self.client_address = self.udp.recvfrom(self.BUFFER_SIZE)
                data = data.decode('utf-8')
                last_read_time = time.time()  # Keep received time
                # Process the received data package
                cmd_splt=data.split(">")[1].split()

                if len(cmd_splt) and cmd_splt[0] == 'odometry':
                    self.odometry = cmd_splt
                if len(cmd_splt) and cmd_splt[0] == 'laserScan':
                    if cmd_splt[2] == '1801':
                        self.laserScanB1.append(cmd_splt)
                        # print(cmd_splt)
                        count = count + 1
                        #print(count)
                    elif cmd_splt[2] == '1802':
                        self.laserScanB4.append(cmd_splt)
                        count = count + 1
                        #print(count)
                if len(cmd_splt) and cmd_splt[0] == 'kmp_statusdata':
                    self.kmp_statusdata = cmd_splt
                if len(cmd_splt) and cmd_splt[0] == 'lbr_statusdata':
                    self.lbr_statusdata = cmd_splt
                if len(cmd_splt) and cmd_splt[0] == 'lbr_sensordata':
                    self.lbr_sensordata.append(cmd_splt)

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
