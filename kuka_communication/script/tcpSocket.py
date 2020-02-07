#!/usr/bin/env python3
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


class TCPSocket:
    def __init__(self, ip, port,node):
        self.BUFFER_SIZE = 4000
        self.isconnected = False
        self.node_name = node
        self.ip = ip
        self.port = port
        self.tcp = None

        #Data
        self.odometry = []
        self.laserScanB1 = []
        self.laserScanB4 = []
        self.kmp_statusdata = None
        self.lbr_statusdata = None

        threading.Thread(target=self.connect_to_socket).start()

    def close(self):
        self.isconnected = False

    def connect_to_socket(self):

        print(cl_cyan('Starting up node:'), self.node_name, 'IP:', self.ip, 'Port:', self.port)
        try:
            self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_address= (self.ip,self.port)
            self.tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
            self.tcp.bind(server_address)
        except:
           print(cl_red('Error: ') + "Connection for KUKA cannot assign requested address:", self.ip, self.port)

        self.tcp.listen(3)
        #print(cl_cyan('Waiting for a connection...'))
        while (not self.isconnected):
            try:
                self.connection, client_address = self.tcp.accept()
                self.tcp.settimeout(0.01)
                self.isconnected = True
            except:
                t=0
        #print(cl_cyan('Connection from: '), client_address)
        time.sleep(1) # wait for FDI

        count = 0
        while self.isconnected:
            try:
                last_read_time = time.time()  # Keep received time
                data = self.recvmsg()
                for pack in (data.decode("utf-8")).split(">"):  # parsing data pack
                    cmd_splt = pack.split()
                    if len(cmd_splt) and cmd_splt[0] == 'odometry':
                        self.odometry = cmd_splt
                        #print('odom')
                    if len(cmd_splt) and cmd_splt[0] == 'laserScan':
                        if cmd_splt[2] == '1801':
                            self.laserScanB1.append(cmd_splt)
                            # print(cmd_splt)
                            #print(count)
                            count = count + 1

                        elif cmd_splt[2] == '1802':
                            self.laserScanB4.append(cmd_splt)
                            count = count + 1
                    if len(cmd_splt) and cmd_splt[0] == 'kmp_statusdata':
                        self.kmp_statusdata = cmd_splt
                    if len(cmd_splt) and cmd_splt[0] == 'lbr_statusdata':
                        self.lbr_statusdata = cmd_splt
                    if len(cmd_splt) and cmd_splt[0] == 'lbr_sensordata':
                        self.lbr_sensordata = self.lbr_sensordata.append(cmd_splt)

            except:
                t = 0
                # elapsed_time = time.time() - last_read_time
                # if elapsed_time > 5.0:  # Didn't receive a pack in 5s
                #  print("exception!!")
                #  self.isconnected = False
                #  print(cl_lightred('No packet received from iiwa for 5s!'))

        print("SHUTTING DOWN")
        self.connection.shutdown(socket.SHUT_RDWR)
        self.connection.close()
        self.tcp.close()
        self.isconnected = False
        print(cl_lightred('Connection is closed!'))
        rclpy.shutdown()


    def send(self, cmd):
        try:
            self.connection.sendall((cmd + '\r\n').encode("UTF-8"))
        except:
            print(cl_red('Error: ') + "sending message thread failed")

    def recvmsg(self):
        header_len = 10
        msglength=0

        byt_len = ""
        byt_len = self.connection.recv(header_len)
        diff_header = header_len - len(byt_len)
        while (diff_header > 0):
            byt_len.extend(self.connection.recv(diff_header))
            diff_header= header_len-len(byt_len)

        msglength = int(byt_len.decode("utf-8")) + 1   #include crocodile and space
        msg = ""

        if(msglength>0 and msglength<5000):
            msg = self.connection.recv(msglength)
            diff_msg = msglength - len(msg)
            while(diff_msg>0):
                print("diff_msg: " + str(diff_msg))
                newmsg = self.connection.recv(diff_msg)
                msg.extend(newmsg)
                diff_msg = msglength - len(msg)
        return msg
