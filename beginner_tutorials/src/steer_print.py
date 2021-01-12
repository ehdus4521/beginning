#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32, Float32
from math import *
import math
import time
import serial
import struct

def steer_print_mode(data):
    #rospy.loginfo("%s",data.data)
    print('steer_angle:', data.data/71)

if __name__ == '__main__':
    rospy.init_node('steer_print')
    rospy.Subscriber('ERP42_steer', Float32, steer_print_mode, queue_size=10)
    
    rate = rospy.Rate(20)
    rospy.spin()