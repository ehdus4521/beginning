#!/usr/bin/env python
# -*- coding: utf-8 -*-

#This is a stauto core_controller.py
#Copyright (c) 2020, choiyungsik, jeonjonghyun

import rospy, roslaunch
import numpy as np
import subprocess
import os
import sys
import rospkg
import math
import time
from enum import Enum
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray, Int32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point

#subscriber
sub_traffic = rospy.Subscriber('/detect/traffic_sign', Int32MultiArray, self.cbTraffic, queue_size=1)
sub_avoidance = rospy.Subscriber('/detect/obstacle', Bool, self.cbAvoidance, queue_size=1)
sub_parking = rospy.Subscriber('/detect/parking_sign', Bool, self.cbParking, queue_size=1)
sub_safetyzone = rospy.Subscriber('/detect/safety_sign', Bool, self.cbSafetyZone, queue_size=1)
sub_crosswalk = rospy.Subscriber('/detect/crosswalk_sign', Bool, self.cdCrosswalk, queue_size=1)
sub_speedbump = rospy.Subscriber('/detect/speedbump_sign', Bool, self.cdSpeedbump, queue_size=1)
sub_stop = rospy.Subscriber('/detect/stop_sign',Bool, self.cbStop,queue_size=1)                       #dynamic obstacle mission
sub_cruise = rospy.Subscriber('/detect/cruise',Bool, self.cbCruise, queue_size=1)

#publisher
pub_state = rospy.Publisher('/state_graph',Int32MultiArray, queue_size=1)
#state = rospy.publish("state_machine", Int32MultiArray, 1)

Machine_State = Enum('Machine_State', 'cruise avoid_cruise stop traffic parking safety_zone crosswalk speedbump')
TrafficSign = Enum('TrafficSign','red green left straightleft')
#self.StopSign = Enum('StopSign','obstacle_stop traffic_stop parking_stop crosswalk_stop')

StateGraph = Int32MultiArray()
StateGraph.layout.dim.append(MultiArrayDimension())
StateGraph.layout.dim[0].label = "state_graph"
StateGraph.layout.dim[0].size = 8
StateGraph.layout.dim[0].stride = 8
StateGraph.layout.data_offset = 0
StateGraph.data=[0]*8
for i in range(8):
    if i == 1:
        self.StateGraph.data[i] = 1
    else:
        self.StateGraph.data[i] = 0

cur_state = self.Machine_State.cruise.value
cur_traffic = self.TrafficSign.green.value
stop_flag = False

loop_rate = rospy.Rate(10)

#class Traffic():
    #def __init__(self):

#class Dyn_obstacle():
    #def __init__(self):

#class Safety_zone():
    #def __init__(self):

class Crosswalk():
    def __init__(self):
        time_end = time.time() + 3
        while time.time() < time_end:
            print('stop')
            Stop.activate()

class Stop():
    def __init__(self):
        self.StateGraph.data[i] = 0
        print('cruise_state : ', self.Machine_State.cruise.value)
        self.StateGraph.data = []

    def activate(self):
        state_graph.[2] = 1
        state.publish(state_graph)

#class Parking():
    #def __init__(self):

#class Speed_bump():
    #def __init__(self):

def main(self):
        rospy.spin()
        # while not rospy.is_shutdown():
        #     self.fnPublishMode()

if __name__ == "__main__":
    rospy.init_node('Core_Controller')
    node = Crosswalk()
    node.main()