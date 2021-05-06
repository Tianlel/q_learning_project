#!/usr/bin/env python3

import rospy, cv2,  cv_bridge
import numpy as np
import keras_ocr
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# HSV color ranges for RGB camera
# [lower range, upper range]
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
HSV_COLOR_RANGES = {
        "blue" : [np.array([110, 100, 100]), np.array([130, 255, 255])],
        "green" : [np.array([50, 100, 100]), np.array([70, 255, 255])],
        "red" : [np.array([0, 100, 100]), np.array([15, 255, 255])]
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

        self.pipeline = keras_ocr.pipeline.Pipeline()

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
        #print("scan callback")
        self.laserscan = data.ranges

    def image_callback(self, data):
        if not self.initialized:
            return
        #print("image callback")
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        self.move_to_block("1")

    def pub_cmd_vel(self, lin, ang):
        self.twist.linear.x = lin
        self.twist.angular.z = ang
        self.cmd_vel_pub.publish(self.twist)

    def move_to_dumbell(self, color):
        if not self.initialized or self.hsv is None or self.image is None or self.laserscan is None:
            return
        # Mask
        mask = cv2.inRange(self.hsv, HSV_COLOR_RANGES[color][0], HSV_COLOR_RANGES[color][1])
        
        
        # Erase all pixels that are not color
        h, w, d = self.image.shape
        
        # determine center of color pixels
        M = cv2.moments(mask)
        if M['m00'] > 0:
            # determine the center of the color pixels in the image
            cx = int(M['m10']/M['m00'])
            print(cx, self.laserscan[0])
            err = w/2 - cx
            k_p = 1.0 / 1000.0
            lin_k = 0.3
            if self.laserscan[0] >= 0.5:
                lin = 0.1
            else:
                err = self.laserscan[0] - 0.3
                lin = err * lin_k
            ang = k_p * err
            self.pub_cmd_vel(lin, ang)
        else:
            print("can't see color")
            "Spin until we see image"
            self.pub_cmd_vel(0, 0.2)
        cv2.imshow("window", mask)
        cv2.waitKey(3)

    def determine_block_center(self, box):
        return box[1][0] - box[0][0]

    def move_to_block(self, block):
        if not self.initialized or self.hsv is None or self.image is None or self.laserscan is None:
            return
        #self.pub_cmd_vel(0, 0)
        images = [self.image]
        prediction_groups = self.pipeline.recognize(images)
        print(prediction_groups[0])
        predictions = dict(prediction_groups[0])
        if block not in predictions:
            self.pub_cmd_vel(0, 0.2)
        else:
            cx = self.determine_block_center(predictions[block].tolist())
            h, w, d = self.image.shape
            err = w/2 - cx
            k_p = 1.0 / 1000.0
            lin_k = 0.3
            if self.laserscan[0] >= 0.5:
                lin = 0.2
            else:
                err = self.laserscan[0] - 0.3
                lin = err * lin_k
            ang = k_p * err
            self.pub_cmd_vel(lin, ang)
        rospy.sleep(2)
        self.pub_cmd_vel(0, 0)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = Action()
    node.run()

