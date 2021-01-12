#! /usr/bin/env python

import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped

if __name__=='__main__':
    rospy.init_node('control')
    # listener = tf.TransformListener()
    ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=10)
    ackermann=AckermannDriveStamped()
    a = [28, 0, -28,0]

while True:
    for angle in range(len(a)):
    #ackermann.drive.steering_angle = (angle * 71 * np.pi / 180)
        ackermann.drive.steering_angle=a[angle]*np.pi/180
        ackermann_pub.publish(ackermann)
        time.sleep(1)