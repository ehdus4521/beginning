#!/usr/bin/env python

import rospy, roslaunch
import numpy as np
import subprocess
import os
import sys
import rospkg
import math
from enum import Enum
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray, Int32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point , PoseStamped

class BackupController():
    def __init__(self):
        
        self.sub_backup = rospy.Subscriber('current_step', PoseStamped, self.cbBackup, queue_size=1)

    def cbBackup(self):

        with open("gps_data_kcity_test_0901_final_1.txt",'r') as file:
            a=file.readlines()
            data=a[1].split(',')
            print(data[0], data[1])

        for i in range (0,80):
            data=a[i].split(',')
            data[0] = float(data[0])
            data[1] = float(data[1])
            print(data[0], data[1])

            if(data[0]==PoseStamped.pose.point.x and data[1]==PoseStamped.pose.point.y): #37.2398614, 126.7732415 37.239364, 126.7732458  37.2389524, 126.7729771
                print("In here")
            else:
                print("Not here")
                #print(data[i][j]', data[i][j])
                # if (data[i][j] == PoseStamped.pose.point.x) and (data[i][j] == PoseStamped.pose.point.y):
                #     print("In here")

        #for i in range (83,88):
        #for i in range (89,92):
        #for i in range (93,107):
        #for i in range (108,114):
        #for i in range (125):
        #for i in range (126,138):
        #for i in range (180,185):
        #for i in range (200,203):
        #for i in range (204,205):
        #for i in range (215,226):
        #for i in range (252,258):
        #for i in range (372,380):
        #for i in range (391,398):

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    
    rospy.init_node('Back_up_Controller')
    node = BackupController()
    node.main()