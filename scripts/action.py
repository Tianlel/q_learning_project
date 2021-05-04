#!/usr/bin/env python3

import rospy, cv2,  cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# HSV color ranges for RGB camera
# [lower range, upper range]
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
HSV_COLOR_RANGES = {
        "red" : [np.array(110, 100, 100), np.array(130, 255, 255)],
        "green" : [np.array(50, 100, 100), np.array(70, 255, 255)],
        "blue" : [np.array(0, 100, 100), np.array(15, 255, 255)]
        }

class Action:
    def __init__(self):
        # set up ROS and CV bridge
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)
        
        # Set image, hsv to be NONE for now
        self.image = None
        self.hsv = None

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

        self.twist = Twist()

    def image_callback(self, msg):
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


