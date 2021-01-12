#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import smach
import smach_ros
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
from geometry_msgs.msg import Point

# define state Cruise
class Cruise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3','outcome4','outcome5'])

        #subscriber
        self.sub_traffic = rospy.Subscriber('/detect/traffic_sign', Int32MultiArray, self.cbTraffic, queue_size=1)
        self.sub_avoidance = rospy.Subscriber('/detect/obstacle', Bool, self.cbAvoidance, queue_size=1)
        self.sub_parking = rospy.Subscriber('/detect/parking_sign', Bool, self.cbParking, queue_size=1)
        self.sub_safetyzone = rospy.Subscriber('/detect/safety_sign', Bool, self.cbSafetyZone, queue_size=1)
        self.sub_crosswalk = rospy.Subscriber('/detect/crosswalk_sign', Bool, self.cdCrosswalk, queue_size=1)
        self.sub_speedbump = rospy.Subscriber('/detect/speedbump_sign', Bool, self.cdSpeedbump, queue_size=1)
        self.sub_stop = rospy.Subscriber('/detect/stop_sign',Bool, self.cbStop,queue_size=1)                       #dynamic obstacle mission
        self.sub_cruise = rospy.Subscriber('/detect/cruise',Bool, self.cbCruise, queue_size=1)

        self.Machine_State = Enum('Machine_State', 'cruise avoid_cruise stop traffic parking safety_zone crosswalk speedbump')
        self.TrafficSign = Enum('TrafficSign','red green left straightleft')
        #self.StopSign = Enum('StopSign','obstacle_stop traffic_stop parking_stop crosswalk_stop')

        self.cur_state = self.Machine_State.cruise.value
        self.cur_state = self.Machine_State.traffic.value
        self.cur_state = self.Machine_State.crosswalk.value
        self.cur_state = self.Machine_State.parking.value
        self.cur_state = self.Machine_State.safety_zone.value
        self.cur_state = self.Machine_State.speedbump.value

        self.cur_traffic = self.TrafficSign.green.value
        self.stop_flag = False

        loop_rate = rospy.Rate(10)

    def cbTraffic(self,event_msg):
        if event_msg.data == True:
            return self.cur_state == self.Machine_State.traffic.value

    def cbStop(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.stop.value,0)

    def cbAvoidance(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.avoid_cruise.value,0)

    def cbParking(self,event_msg):
        if event_msg.data == True:
            return self.cur_state == self.Machine_State.parking.value

    def cbCruise(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.cruise.value,0)

    def cbSafetyZone(self,event_msg):
        if event_msg.data == True:
            return self.cur_state == self.Machine_State.safety_zone.value

    def cdCrosswalk(self,event_msg):
        if event_msg.data == True:
            return self.cur_state == self.Machine_State.crosswalk.value

    def cdSpeedbump(self,event_msg):
        if event_msg.data == True:
            return self.cur_state == self.Machine_State.speedbump.value
            
    def execute(self, userdata):
        rospy.loginfo('Executing State Cruise_mode')
        if self.cur_state == self.Machine_State.traffic.value:
            return 'outcome1'
        elif self.cur_state == self.Machine_State.crosswalk.value:
            return 'outcome2'
        elif self.cur_state == self.Machine_State.parking.value:
            return 'outcome3'
        elif self.cur_state == self.Machine_State.safety_zone.value:
            return 'outcome4'
        elif self.cur_state == self.Machine_State.speedbump.value:
            return 'outcome5'
#--------------------------------------------------------------------------------------------------------
# define state traffic
class Traffic(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome6'])

    def execute(self, userdata):
        rospy.loginfo('Executing State Traffic_mode')
        return 'outcome6'
        
# define state crosswalk
class Crosswalk(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome7'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Crosswalk_mode')
        return 'outcome7'

# define state parking
class Parking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome8'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Parking_mode')
        return 'outcome8'

# define state safety_zone
class SafetyZone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome9'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SafetyZone_mode')
        return 'outcome9'

# define state speedbump
class Speedbump(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome10'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Speedbump_mode')
        return 'outcome10'
#--------------------------------------------------------------------------------------------------------
def main():
    rospy.init_node('Core_Controller')
    # Create a Core_Controller
    sm = smach.StateMachine(outcomes=['outcome11'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Cruise_mode', Cruise(), 
                               transitions={'outcome1':'Traffic_mode', 'outcome2':'Crosswalk_mode', 'outcome3':'Parking_mode','outcome4':'SafetyZone_mode','outcome5':'Speedbump_mode'})
        smach.StateMachine.add('Traffic_mode', Traffic(), 
                               transitions={'outcome6':'outcome11'})
        smach.StateMachine.add('Crosswalk_mode', Crosswalk(), 
                               transitions={'outcome7':'outcome11'})
        smach.StateMachine.add('Parking_mode', Parking(), 
                               transitions={'outcome8':'outcome11'})
        smach.StateMachine.add('SafetyZone_mode', SafetyZone(), 
                               transitions={'outcome9':'outcome11'})
        smach.StateMachine.add('Speedbump_mode', Speedbump(), 
                               transitions={'outcome10':'outcome11'})    

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    # while not rospy.is_shutdown():
    #     self.fnPublishMode()

if __name__ == "__main__":
    main()