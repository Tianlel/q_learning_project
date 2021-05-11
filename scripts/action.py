#!/usr/bin/env python3

import rospy, cv2,  cv_bridge
import numpy as np
import keras_ocr
import moveit_commander
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import RobotMoveDBToBlock, NodeStatus

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
                    '2' : ['2', 'z'],
                    '3' : ['3', '8', 'e', '5', 's']
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

        # initialize node status publisher 
        self.node_status_pub = rospy.Publisher("node_status", NodeStatus, queue_size=10)
        
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

        # Initial state to STOP
        self.state = STOP

        # Keep track of direction robot last turned for driving to block
        self.last_turn = None
        
        # Keep track of whether the correct block has been spotted at least once (for driving to block)
        self.block_visible = False
        
        # Store actions in list as they are recieved in the callback
        self.actions = []
        
        # Current action
        self.action_in_progress = None

        # Set up publisher for movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

        self.twist = Twist()

        # Set gripper to initial starting point
        self.reset_gripper()  

        self.initialized = True

        # tell dispatch_actions node to start publish actions
        self.node_status_pub.publish(NodeStatus(node_initialized=True))
        rospy.sleep(1)

    """Callback for lidar scan"""
    def scan_callback(self, data):
        if not self.initialized:
            return
        self.laserscan = data.ranges
        front = []
        for i in range(350, 375):
            front.append(self.laserscan[i % 360])
        self.laserscan_front = front

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
        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

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
            # spin until we see image
            self.pub_cmd_vel(0, 0.2)

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
        # Move back away from dumbells
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 1.0:
            self.pub_cmd_vel(-0.3, 0)
        self.pub_cmd_vel(0,0)
        # Change state to block  
        block = self.action_in_progress.block_id
        if block == 1:
            self.state = BLOCK1
        elif block == 2:
            self.state = BLOCK2
        else:
            self.state = BLOCK3
        

    """Drop dumbell from gripper"""
    def drop_gripper(self):
        arm_joint_goal = [0.0, 0.43, 0.48, -0.92]
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        
        init_time = rospy.Time.now().to_sec()
        # move back away from block
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 3.0:
            self.pub_cmd_vel(-0.3, 0)
        self.pub_cmd_vel(0,0)
        self.reset_gripper() 
        # Change state, reset variables used for driving to block
        self.state = STOP
        self.block_visible = False
        self.action_in_progress = None
        self.last_turn = None
    

    """Determine if there are dumbells in box"""
    def check_for_dumbells(self, box, hsv):
        # Masks
        redmask = cv2.inRange(hsv, HSV_COLOR_RANGES['red'][0], HSV_COLOR_RANGES['red'][1])            
        bluemask = cv2.inRange(hsv, HSV_COLOR_RANGES['blue'][0], HSV_COLOR_RANGES['blue'][1]) 
        greenmask = cv2.inRange(hsv, HSV_COLOR_RANGES['red'][0], HSV_COLOR_RANGES['red'][1])
        # Check for an all black box meaning there are no dumbells near the block
        if cv2.countNonZero(redmask) == 0 and cv2.countNonZero(bluemask) == 0 and cv2.countNonZero(greenmask) == 0:
            return False
        else:
            return True
         


    """Determine center of block based on predictions from keras_ocr"""
    def determine_block_center(self, boxes_to_use):
        # Keep track of number of blocks and x-positions
        sum_boxes = 0
        count_boxes = 0
        for box in boxes_to_use:
            sum_boxes += box[1][0] + box[0][0]
            count_boxes += 2
        return sum_boxes / count_boxes
    
    """Move to specified blocks states: BLOCK1 BLOCK2 BLOCK3"""
    def move_to_block(self, block):
        if not self.initialized or self.hsv is None or self.image is None or self.laserscan is None:
            return
        # Make sure robot is stopped
        self.pub_cmd_vel(0, 0)
        # front distance
        dist = min(self.laserscan_front)
        
        # Time to drop the block
        if dist <= 0.5 and self.block_visible:
            self.state = DROP
            self.pub_cmd_vel(0.0, 0.0)
            return
        
        # Set up for keras_ocr, check_for_dumbells, determine_block_center
        image = self.image
        hsv = self.hsv
        images = [image]
        prediction_groups = self.pipeline.recognize(images)
        predictions = prediction_groups[0]
        boxes_to_use = [] # which blocks to use for determining where to drive
        block_in_prediction = False # bool variable to tell if any blocks have been chosen 

        # iterate through all predictions and box tuples
        for p in predictions:
            box = p[1].astype(int)
            # check if there is a dumbell near the block
            dumbell_near_box = self.check_for_dumbells(box, hsv)
            if p[0] == block and not dumbell_near_box:
                # We have seen the correct block and there is no dumbell near it 
                # The first time we see the block we require an exact character match
                self.block_visible = True
            if p[0] in possible_boxes[block] and not dumbell_near_box:
                # There are blocks to use in the prediction
                # Add blocks we want to use
                block_in_prediction = True
                boxes_to_use.append(box)
        # We have seen the correct block, meaning we are pointed near it. 
        # We recieved a bad prediction, meaning we may have turned too far
        # So go forward slightly and try to readjust by turning the opposite direction from before
        if self.block_visible and not block_in_prediction:
            lin = 0.1
            if self.last_turn == 'right':    
                ang = 0.4
            elif self.last_turn == 'left':
                ang = -0.4
        elif not self.block_visible or not block_in_prediction:
            # We have not yet seen the correct block so keep turning
            ang = 0.5
            lin = 0.0
        else:
            # Proportional control for if we have boxes to use
            cx = self.determine_block_center(boxes_to_use)
            h, w, d = self.image.shape
            err = w/2 - cx
            if dist >= 1.0:
                k_p = 1.0 / 400.0
            else:
                k_p = 1.0 / 500.0
            lin_k = 0.3
            if dist >= 1.0:
                lin = 0.4
            else:
                linerr = dist - 0.5
                lin = linerr * lin_k
            ang = k_p * err
        # Determine last direction turned
        if ang > 0:
            self.last_turn = 'left'
        else:
            self.last_turn = 'right'
        # Perform movement for a small amount of time
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < 0.75:
            self.pub_cmd_vel(lin, ang)
        self.pub_cmd_vel(0,0)
        
    
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

    """The driver of our node, calls functions dpending on state"""
    def run(self):
        r = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.initialized:
                if self.state == BLOCK1 or self.state == BLOCK2 or self.state == BLOCK3:
                    # Move to block
                    self.move_to_block(self.state)
                elif self.state == GREEN or self.state == BLUE or self.state == RED:
                    # Go to dumbell color
                    self.move_to_dumbell(self.state)
                elif self.state == PICKUP:
                    # Pick up dumbell
                    self.pick_up_gripper()
                elif self.state == DROP:
                    # Drop dumbell
                    self.drop_gripper()
                elif self.state == STOP:
                    # Done with action, wait for next one
                    self.get_action()
            r.sleep()


if __name__ == '__main__':
    # Declare a node and run it.
    node = Action()
    node.run()

