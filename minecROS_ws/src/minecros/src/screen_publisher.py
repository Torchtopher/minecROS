#!/usr/bin/env python3
import cv2
import numpy
import time
import mss
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
    
# initalize node
rospy.init_node('screen_pub', anonymous=True)
pub = rospy.Publisher('screen_img', Image, queue_size=1)
br = CvBridge()
rospy.loginfo("Starting screen image publisher")

with mss.mss() as sct:
    while not rospy.is_shutdown():
        # Get raw pixels from the screen, save it to a Numpy array
        monitor = sct.monitors[1]
        img = numpy.array(sct.grab(monitor))
        #print(img.shape)
        pub.publish(br.cv2_to_imgmsg(img))