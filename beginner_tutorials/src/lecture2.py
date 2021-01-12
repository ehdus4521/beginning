#! /usr/bin/env python

import rospy ,cv2, time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def image_callback(data):
    global sub_image

    sub_image=data
    
if __name__ == '__main__':

    rospy.init_node("camera_sub")

    rospy.Subscriber("image",Image, image_callback)

    bridge = CvBridge()

    sub_image=Image()

    time.sleep(1)

    while not rospy.is_shutdown():

        final_image=bridge.imgmsg_to_cv2(sub_image,"bgr8")

        cv2.imshow("bbb",final_image)

        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    else:
        pass