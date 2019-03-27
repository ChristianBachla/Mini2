#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('video_background_node')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class BackgroundSub(object):
    """docstring forBackgroundSub."""

    def __init__(self):
        self.count = 0
        self.fgbg = cv2.createBackgroundSubtractorMOG2(300, 50, False)
        rospy.init_node('background')
        self.background_pub = rospy.Publisher("background", Image, queue_size=10)
        self.bridge = CvBridge()
        self.kernel = np.ones((4, 4), np.uint8)
        rospy.Subscriber('analyzed_image', Image, self.callback)
        #rospy.spin()

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        image = self.fgbg.apply(cv_image)
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, self.kernel)
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, self.kernel)

        #image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        try:
            self.background_pub.publish(self.bridge.cv2_to_imgmsg(image, "mono8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    bg = BackgroundSub()
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
#    BackgroundSub()
#    print("Launching the background subtracter")
    main(sys.argv)
