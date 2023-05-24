#!/usr/bin/env python3
import cv2
import numpy
import time
import mss
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from PIL import Image as im
import numpy as np

# initalize node
rospy.init_node('screen_pub', anonymous=True)
pub = rospy.Publisher('screen_img', Image, queue_size=1)
br = CvBridge()
rospy.loginfo("Starting screen image publisher")

with mss.mss() as sct:
    while not rospy.is_shutdown():
        # Get raw pixels from the screen, save it to a Numpy array
        monitor = sct.monitors[1]
        img = sct.grab(monitor)
        img = im.frombytes("RGB", img.size, img.bgra, "raw", "BGRX")
        pub.publish(br.cv2_to_imgmsg(np.array(img), encoding="passthrough"))