#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('tracker_node')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Tracker(object):
    """docstring forTracker."""

    def __init__(self):
        self.count = 0
        rospy.init_node('tracker')
        self.tracker_pub = rospy.Publisher("tracker", Image, queue_size=10)
        self.bridge = CvBridge()
        self.kernel = np.ones((4, 4), np.uint8)
        rospy.Subscriber('analyzed_image', Image, self.callback)
        rospy.spin()

    def callback(self, data):
        lk_params = dict(winSize=(10, 10), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        if self.count == 0:
            self.old_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            self.x = 330
            self.y = 1000
            self.old_points = np.array([[self.x, self.y]], dtype=np.float32)
            self.count = 1

        #print(self.x, self.y)
        new_points, status, error = cv2.calcOpticalFlowPyrLK(self.old_image, cv_image, self.old_points, None, **lk_params)

        self.old_image = cv_image.copy()
        self.old_points = new_points

        self.x, self.y = new_points.ravel()

        image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        cv2.circle(image, (330, 1000), 15, 255, 3)
        cv2.circle(image, (self.x, self.y), 10, (0, 0, 255), 2)
        cv2.putText(image, 'X:' + str(int(self.x))+' Y:' + str(int(self.y)), (self.x, self.y), cv2.FONT_HERSHEY_SIMPLEX, 0.8,(255,255,255),1,cv2.LINE_AA)

        try:
            self.tracker_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    Tracker()
#    print("Launching the background subtracter")
#    main(sys.argv)
