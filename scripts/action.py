#!/usr/bin/env python3

import rospy, cv2,  cv_bridge
import numpy as np
import keras_ocr
import moveit_commander
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import RobotMoveDBToBlock

# HSV color ranges for RGB camera
# [lower range, upper range]
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
HSV_COLOR_RANGES = {
        "blue" : [np.array([110, 100, 100]), np.array([130, 255, 255])],
        "green" : [np.array([50, 100, 100]), np.array([70, 255, 255])],
        "red" : [np.array([0, 100, 100]), np.array([15, 255, 255])]
        }

class Action(object):
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

        # subscribe to robot_action
        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.execute_action)
        
        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        #self.pipeline = keras_ocr.pipeline.Pipeline()

        # Set image, hsv, scan data to be NONE for now
        self.image = None
        self.hsv = None
        self.laserscan = None
        self.laserscan_front = None
        self.state = 'GREEN'

        self.action_in_progress = False

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

        self.twist = Twist()

        self.reset_gripper()  

        rospy.sleep(1)
        self.initialized = True

    def scan_callback(self, data):
        if not self.initialized:
            return
        #print("scan callback")
        self.laserscan = data.ranges
        front = []
        for i in range(355, 365):
            front.append(self.laserscan[i % 360])
        self.laserscan_front = front

    def image_callback(self, data):
        if not self.initialized:
            return
        #print("image callback")
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # TESTING FOR NOW
        print(self.state)
        if self.state == 'BLOCK1':
            self.move_to_block('1')
        if self.state == 'GREEN':
            self.move_to_dumbell("green")
        if self.state == 'PICKUP':
            self.pick_up_gripper()
        if self.state == 'STOP':
            rospy.sleep(2)
            self.drop_gripper()

    # Publish movement
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
        
        # front distance
        dist = min(self.laserscan_front)

        # determine center of color pixels
        M = cv2.moments(mask)
        if M['m00'] > 0:
            # determine the center of the color pixels in the image
            cx = int(M['m10']/M['m00'])
            print(cx, dist)
            err = w/2 - cx
            k_p = 1.0 / 1000.0
            lin_k = 0.3
            if dist <= 0.22:
                self.state = 'PICKUP'
                self.pub_cmd_vel(0, 0)
                return
            if dist >= 0.5:
                lin = 0.1
            else:
                linerr = dist - 0.22
                lin = linerr * lin_k
            ang = k_p * err
            self.pub_cmd_vel(lin, ang)
        else:
            print("can't see color")
            "Spin until we see image"
            self.pub_cmd_vel(0, 0.2)
        cv2.imshow("window", mask)
        cv2.waitKey(3)

    # Starting gripper setup to pickup
    def reset_gripper(self):
        arm_joint_goal = [0.0, 0.7, -0.260, -0.450]
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
    
    # Gripper with dumbell
    def pick_up_gripper(self):
        arm_joint_goal = [0.0, 0.10, -0.5, -0.2]
        gripper_joint_goal = [0.004, 0.004]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True) 
        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        self.state = 'STOP'
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 1.0:
            self.pub_cmd_vel(-0.3, 0)
        self.pub_cmd_vel(0,0)

    # Drop dumbell from gripper
    def drop_gripper(self):
        arm_joint_goal = [0.0, 0.7, -0.260, -0.450]
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        rospy.sleep(1)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 1.0:
            self.pub_cmd_vel(-0.3, 0)
        self.pub_cmd_vel(0,0)

    
    def determine_block_center(self, box):
        return box[1][0] - box[0][0]
    """
    def move_to_block(self, block):
        if not self.initialized or self.hsv is None or self.image is None or self.laserscan is None:
            return
        self.pub_cmd_vel(0, 0)
        
        # front distance
        dist = min(self.laserscan_front)

        images = [self.image]
        prediction_groups = self.pipeline.recognize(images)
        print(prediction_groups[0])
        predictions = dict(prediction_groups[0])
        cv2.imshow("window", self.image)
        cv2.waitKey(3)

        
        if block not in predictions:
            ang = 0.3
            lin = 0.0
        else:
            cx = self.determine_block_center(predictions[block].tolist())
            h, w, d = self.image.shape
            err = w/2 - cx
            k_p = 1.0 / 1000.0
            lin_k = 0.2
            if dist >= 0.5:
                lin = 0.2
            else:
                linerr = dist - 0.2
                lin = linerr * lin_k
            ang = k_p * err
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 1.0:
            self.pub_cmd_vel(lin, ang)
        self.pub_cmd_vel(0,0)
        """
    
    # execute action
    def execute_action(self, data):
        # TODO
        return

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = Action()
    node.run()

