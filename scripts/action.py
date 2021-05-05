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
        "red" : [np.array([110, 100, 100]), np.array([130, 255, 255])],
        "green" : [np.array([50, 100, 100]), np.array([70, 255, 255])],
        "blue" : [np.array([0, 100, 100]), np.array([15, 255, 255])]
        }

class Action:
    def __init__(self):
        self.initialized = False
        # init rospy node
        rospy.init_node("action")
        
        # set up ROS and CV bridge
        self.bridge = cv_bridge.CvBridge()

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)
       
        # subscribe to robot's laser scan
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Set image, hsv, scan data to be NONE for now
        self.image = None
        self.hsv = None
        self.laserscan = None

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

        self.twist = Twist()
        rospy.sleep(1)
        self.initialized = True

    def scan_callback(self, data):
        if not self.initialized:
            return
        print("scan callback")
        self.laserscan = data.ranges

    def image_callback(self, data):
        if not self.initialized:
            return
        print("image callback")
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

    def pub_cmd_vel(self, lin, ang):
        self.twist.linear.x = lin
        self.twist.angular.z = ang
        self.cmd_vel_pub.publish(self.twist)

    def go_to_dumbell(self, color):
        # Mask
        mask = cv2.inRange(hsv, HSV_COLOR_RANGES[color][0], HSV_COLOR_RANGES[color][1])
        
        # Erase all pixels that are not color
        h, w, d = image.shape
        search_top = int(3*h/4)
        search_bot = int(3*h/4 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        
        # determine center of color pixels
        M = cv2.moments(mask)
        
        if M['m00'] > 0:
            # determine the center of the color pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            err = w/2 - cx
            k_p = 1.0 / 100.0
            lin = 0.2
            ang = k_p * err
            self.pub_cmd_vel(lin, ang)


    def run(self):
        self.go_to_dumbell("red")
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = Action()
    node.run()
