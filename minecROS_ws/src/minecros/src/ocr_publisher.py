#!/usr/bin/env python3

import pytesseract
from PIL import Image
import cv2
import numpy as np
from pascal import PascalVOC
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import rospkg 
import os
import time
from PIL import Image as im
import io

class MinecROSOCR:

    def __init__(self) -> None:

        
        rospack = rospkg.RosPack()
        self.THIS_DIR = os.path.join(rospack.get_path('minecros'), 'config/')
        PATH_TO_LABELS = os.path.join(self.THIS_DIR, 'coords.xml')
        
        boxes = PascalVOC.from_xml(PATH_TO_LABELS)
        self.coord_x = 0
        
        for box in boxes.objects:
            if box.name == "coords":
                self.coord_x = box.bndbox.xmin
                self.coord_y = box.bndbox.ymin
                self.coord_w = box.bndbox.xmax - box.bndbox.xmin
                self.coord_h = box.bndbox.ymax - box.bndbox.ymin
        # make sure coords are found
        if self.coord_x == 0:
            rospy.logerr(f"No element with name 'coords' found in annotation file config.xml. Please rerun the config and check the filename is correct")
            exit(1)
    
        self.img_sub = rospy.Subscriber("/autofarm/screen_img", Image, self.image_CB)
        self.coord_pub = rospy.Publisher("/minecros/coords", Point, queue_size=1)
        self.cv_bridge = CvBridge()

    
    def image_CB(self, msg):
        # convert image to cv2 format
        image = self.cv_bridge.imgmsg_to_cv2(msg)

        # crop image
        image = image[self.coord_y:self.coord_y+self.coord_h, self.coord_x:self.coord_x+self.coord_w]
        image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2HSV)
        hsv_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2HSV)
        color_lower = np.array([100, 255, 220])
        color_upper = np.array([200, 255, 230])
        mask = cv2.inRange(hsv_image, color_lower, color_upper)
        result = cv2.bitwise_and(image, image, mask=mask)
        image = cv2.cvtColor(np.array(result), cv2.COLOR_RGB2GRAY)
        image = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

        text = pytesseract.image_to_string(image, lang='mc', config='-c tessedit_char_whitelist=0123456789')
        print(text)
        # looks like XX X1X1\nYY Y1Y1 ZZ Z1Z1
        if text.count(" ") == 4:
            print("found coords")
            coords = text.replace("\n", " ").split(" ")
            # remove whatever \x0c is
            coords = [coord.replace("\x0c", "") for coord in coords]
            # remove empty strings
            coords = list(filter(None, coords))
            print(coords)
            # add pair elements together as string with decimal point
            coords = [coords[i] + "." + coords[i+1] for i in range(0, len(coords), 2)]
            coords = [float(coord) for coord in coords]
            msg = Point()
            msg.x = coords[0]
            msg.y = coords[1]
            msg.z = coords[2]
            self.coord_pub.publish(msg)
        else:
            rospy.logwarn_throttle(2, "No coords found from OCR")
        

# initalize node
if __name__ == "__main__":
    rospy.init_node('minecros_OCR', anonymous=True)
    rospy.loginfo("Starting OCR node")
    ocr = MinecROSOCR()
    while not rospy.is_shutdown():
        rospy.spin()