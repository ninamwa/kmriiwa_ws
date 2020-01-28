#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import sys

#from script.test import Test
#from script.tcpSocket import *
from scripts import *


class Test2():
    def __init__(self,ip):
        print('hei2hei2')
        print('nina')
        m = Test()

        n = TCPSocket('1234',10)
        #print(n)

def main():

  print('HEI')


if __name__ == '__main__':
    main()