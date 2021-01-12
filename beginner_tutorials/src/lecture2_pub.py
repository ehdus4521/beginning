#! /usr/bin/env python

import rospy ,cv2, time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

if __name__ == '__main__':

    rospy.init_node("camera_pub")

    capture=cv2.VideoCapture(0)

    capture.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

    video_pub=rospy.Publisher("image",Image,queue_size=5)

    bridge = CvBridge()

    while not rospy.is_shutdown():

        if (capture.isOpened()):
            ret, frame=capture.read()

            cv2.imshow('a',frame)

            image=bridge.cv2_to_imgmsg(frame,"bgr8")

            video_pub.publish(image)

        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    else:
        pass