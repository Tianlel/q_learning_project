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

# What keras_ocr may see for each number
possible_boxes = {
                    '1' : ['1', 'i', 'l'],
                    '2' : ['2'],
                    '3' : ['3', '8', 'e']
                }

# List of possible states
STOP = 'stop' # robot not doing anything, either waiting for action or done
# Go to dumbell color
GREEN = 'green'
BLUE = 'blue'
RED = 'red'
# Go to block number 
BLOCK1 = '1'
BLOCK2 = '2'
BLOCK3 = '3'
# Gripper actions
PICKUP = 'pickup'
DROP = 'drop'

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
        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.action_callback)
        
        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        self.pipeline = keras_ocr.pipeline.Pipeline()

        # Set image, hsv, scan data to be NONE for now
        self.image = None
        self.hsv = None
        self.laserscan = None
        self.laserscan_front = None
        self.state = STOP
        
        self.block_visible = False
        
        # Testing for now
        self.actions = [RobotMoveDBToBlock('green', 2)]

        self.action_in_progress = None

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

        self.twist = Twist()

        self.reset_gripper()  

        self.initialized = True
        rospy.sleep(1)

    """Callback for lidar scan"""
    def scan_callback(self, data):
        if not self.initialized:
            return
        self.laserscan = data.ranges
        front = []
        for i in range(355, 365):
            front.append(self.laserscan[i % 360])
        self.laserscan_front = front
        # Doing this in SCAN callback becuase it is slower and gives more time for image to load.
        if self.state == BLOCK1 or self.state == BLOCK2 or self.state == BLOCK3:
            self.move_to_block(self.state)

    """Callback for actions"""
    def action_callback(self, data):
        if not self.initialized:
            return
        #if data is not None:
        self.actions.append(data)

    """Callback for images"""
    def image_callback(self, data):
        if not self.initialized:
            return
        #print("image callback")
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # Main driver
        #print(self.state)
        if self.state == GREEN or self.state == BLUE or self.state == RED:
            # Go to dumbell color
            self.move_to_dumbell(self.state)
        elif self.state == PICKUP:
            self.pick_up_gripper()
        elif self.state == DROP:
            self.drop_gripper()
        elif self.state == STOP:
            self.get_action()

                
    """Publish movement"""
    def pub_cmd_vel(self, lin, ang):
        self.twist.linear.x = lin
        self.twist.angular.z = ang
        self.cmd_vel_pub.publish(self.twist)

    """Move to dumbell of particular color, states = GREEN, BLUE, RED"""
    def move_to_dumbell(self, color):
        if not self.initialized or self.hsv is None or self.image is None or self.laserscan is None:
            return

        # Mask
        mask = cv2.inRange(self.hsv, HSV_COLOR_RANGES[color][0], HSV_COLOR_RANGES[color][1])            
        
        # Dimensions
        h, w, d = self.image.shape
        
        # front distance
        dist = min(self.laserscan_front)

        # determine center of color pixels
        M = cv2.moments(mask)
        # if the target color is in sight
        if M['m00'] > 0:
            # determine the center of the color pixels in the image
            cx = int(M['m10']/M['m00'])
            err = w/2 - cx # error
            k_p = 1.0 / 1000.0 # angular proportional control
            lin_k = 0.3 # linear proportional control
            # Time to pickup and change state
            if dist <= 0.22:
                self.state = PICKUP
                self.pub_cmd_vel(0, 0)
                return
            # All proportional control
            if dist >= 0.5:
                lin = 0.2
            else:
                linerr = dist - 0.22
                lin = linerr * lin_k
            ang = k_p * err
            self.pub_cmd_vel(lin, ang)
        else:
            print("can't see color")
            # spin until we see image
            self.pub_cmd_vel(0, 0.2)
        #cv2.imshow("window", mask)
        #cv2.waitKey(3)

    """Starting gripper setup to pickup"""
    def reset_gripper(self):
        arm_joint_goal = [0.0, 0.7, -0.260, -0.450]
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
    
    """Gripper with dumbell"""
    def pick_up_gripper(self):
        arm_joint_goal = [0.0, 0.10, -0.5, -0.2]
        gripper_joint_goal = [0.004, 0.004]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_gripper.go(gripper_joint_goal, wait=True) 
        self.move_group_arm.stop()
        self.move_group_gripper.stop()
        print("done picking up, moving away")
        # Move back away from dumbells
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 1.0:
            self.pub_cmd_vel(-0.3, 0)
        self.pub_cmd_vel(0,0)
        # Change state
        print("changing state")
        block = self.action_in_progress.block_id
        if block == 1:
            self.state = BLOCK1
        elif block == 2:
            self.state = BLOCK2
        else:
            self.state = BLOCK3


    """Drop dumbell from gripper"""
    def drop_gripper(self):
        arm_joint_goal = [0.0, 0.7, -0.260, -0.450]
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        rospy.sleep(1)
        init_time = rospy.Time.now().to_sec()
        # move back away from block
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 2.0:
            self.pub_cmd_vel(-0.5, 0)
        self.pub_cmd_vel(0,0)
        self.reset_gripper()
        # Change state
        self.state = 'STOP'
        self.block_visible = False
        self.action_in_progress = None

    
    """Determine center of block based on predictions from keras_ocr"""
    def determine_block_center(self, block_num, prediction_groups):
        # Possible recognitions for 
        sum_boxes = 0
        count_boxes = 0
        possibilities = []
        for i in prediction_groups:
            if i in possible_boxes[block_num]:
                possibilities.append(i)
                sum_boxes += prediction_groups[i][1][0] + prediction_groups[i][0][0]
                count_boxes += 2
        #print(possibilities)
        #print(sum_boxes, count_boxes)
        return sum_boxes / count_boxes
    
    """Move to specified blocks states: BLOCK1 BLOCK2 BLOCK3"""
    def move_to_block(self, block):
        if not self.initialized or self.hsv is None or self.image is None or self.laserscan is None:
            return
        self.pub_cmd_vel(0, 0)
        
        # front distance
        dist = min(self.laserscan_front)
        
        image = self.image
        images = [image]
        prediction_groups = self.pipeline.recognize(images)
        predictions = dict(prediction_groups[0])
        print(predictions)
        print(dist)
        #cv2.imshow("window", self.image)
        #cv2.waitKey(3)
        block_in_prediction = False
        for pic in predictions:
            if pic == block:
                self.block_visible = True
            if pic in possible_boxes[block]:
                block_in_prediction = True
        if dist <= 3.0 and self.block_visible:
            self.state = 'DROP'
            self.pub_cmd_vel(0.0, 0.0)
            return
        if self.block_visible and not block_in_prediction:
            lin = 0.2
            ang = 0.5
        elif not self.block_visible:
            ang = 0.5
            lin = 0.0
        else:
            cx = self.determine_block_center(block, predictions)
            h, w, d = self.image.shape
            err = w/2 - cx
            print(err, cx, w)
            k_p = 1.0 / 200.0
            lin_k = 0.3
            if dist >= 3.0:
                lin = 0.5
            else:
                linerr = dist - 1.0
                lin = linerr * lin_k
            ang = k_p * err
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 1.0:
            self.pub_cmd_vel(lin, ang)
        self.pub_cmd_vel(0,0)
        rospy.sleep(0.5)
        
    
    # execute action
    def get_action(self):
        if not self.initialized or self.hsv is None or self.image is None or self.laserscan is None or not self.actions:
            return
        
        # Pop the next action to be done
        self.action_in_progress = self.actions.pop(0)
        # Get color of dumbell to go to and set state accordingly
        color = self.action_in_progress.robot_db
        if color == 'red':
            self.state = RED
        elif color == 'blue':
            self.state = BLUE
        else:
            self.state = GREEN
        return

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = Action()
    node.run()

