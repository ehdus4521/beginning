#!/usr/bin/env python

import rospy
import serial
import time
import tf
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

if __name__ == '__main__':

    rospy.init_node("IMU")

    ser=serial.serial_for_url("/dev/ttyUSB0",115200,timeout=0)
    imu_pub=rospy.Publisher("imu",Imu,queue_size=5)

    prev_rpy=np.zeros(3)

    rpy=np.zeros(3)
    vel_rpy=np.zeros(3)
    accel=np.zeros(3)

    prev_time=time.time()

    imumsg=Imu()

    while not rospy.is_shutdown():
        imu_message=ser.readline()
        
        if (imu_message[:1]=='*'):
            data=imu_message.split(',')

            rpy[0]=round(float(data[0][1:]),3)
            rpy[1]=round(float(data[1]),3)
            rpy[2]=round(float(data[2]),3)

            quaternion = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            print(quaternion)

            dt=time.time()-prev_time
            
            vel_rpy[0]=(rpy[0]-prev_rpy[0])/dt
            vel_rpy[1]=(rpy[1]-prev_rpy[1])/dt
            vel_rpy[2]=(rpy[2]-prev_rpy[2])/dt

            prev_time=time.time()

            prev_rpy[0]=rpy[0]
            prev_rpy[1]=rpy[1]
            prev_rpy[2]=rpy[2]

            accel[0]=round(float(data[3]),3)
            accel[1]=round(float(data[4]),3)
            accel[2]=round(float(data[5]),3)

            imumsg.header.stamp=rospy.Time.now()
            imumsg.header.frame_id="imu_link"

            imumsg.orientation.x=quaternion[0]
            imumsg.orientation.y=quaternion[1]
            imumsg.orientation.z=quaternion[2]
            imumsg.orientation.w=quaternion[3]

            imumsg.angular_velocity.x=vel_rpy[0]
            imumsg.angular_velocity.y=vel_rpy[1]
            imumsg.angular_velocity.z=vel_rpy[2]

            imumsg.linear_acceleration.x=accel[0]
            imumsg.linear_acceleration.x=accel[1]
            imumsg.linear_acceleration.x=accel[2]

            imu_pub.publish(imumsg)

            #print(rpy[0],rpy[1],rpy[2],accel[0],accel[1],accel[2])
    else:
        pass