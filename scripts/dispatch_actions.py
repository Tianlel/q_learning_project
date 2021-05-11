#!/usr/bin/env python3

""" 
Given the converged Q-matrix, this node publishes actions corresponding 
to the path most likely to lead to receiving a reward
"""

import rospy
import csv
import os
import numpy as np
from q_learning_project.msg import RobotMoveDBToBlock, NodeStatus

# Path of directory on where action_state file is located
path_prefix_action_state = os.path.dirname(__file__) + "/action_states/"
# Path of directory on where Qmatrix file is located
path_prefix_qmatrix = os.path.dirname(__file__) 

class DispatchAction(object):

    def __init__(self):
        rospy.init_node("dispatch_actions")
        rospy.sleep(1)

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix_action_state + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix_action_state + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))

        # Set up action publisher
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)
        rospy.sleep(1)

        # Subscribe to node_status
        self.action_node_status = rospy.Subscriber("node_status", NodeStatus, self.node_status_callback)

        # Initialize action
        self.action = RobotMoveDBToBlock()

        self.action_node_initialized = False

        self.reach_end = False

        # Initialize Q matrix
        self.qmatrix = []

        # Initialize state to 0
        self.state = 0

    def node_status_callback(self, data):
        # if action node is successfully initialized
        if data:
            self.action_node_initialized = True

    # Read the saved Q matrix
    def read_qmatrix(self):
        with open(path_prefix_qmatrix + '/qmatrix.csv', newline='') as csvfile:
            csvreader = csv.reader(csvfile, delimiter=',')
            for state in csvreader:
                self.qmatrix.append([int(action) for action in state])
            print("Loaded Q matrix")
        return

    # Find the move with largest reward; if there are multiple, choose the first one. Then publish the action
    def publish_action(self):      
        max_reward = max(self.qmatrix[self.state])
        print("max reward: "+ str(max_reward))
        # If reaches end state
        if max_reward == 0:
            self.reach_end = True
            return

        best_action = self.qmatrix[self.state].index(max_reward)
        print("best action: "+str(best_action))

        action = self.actions[best_action]

        # Publish the action
        self.action.robot_db = action.get("dumbbell")
        self.action.block_id = action.get("block")
        self.action_pub.publish(self.action)

        print(np.where(self.action_matrix[self.state] == best_action))

        # Update robot state
        self.state = np.where(self.action_matrix[self.state] == best_action)[0][0]
    
    def run(self):
        self.read_qmatrix()
        
        # If file read successfully
        if self.qmatrix:
            while not self.reach_end:
                if self.action_node_initialized:
                    self.publish_action()
                    rospy.sleep(1)

if __name__ == "__main__":
    node = DispatchAction()
    node.run()
