#!/usr/bin/env python3

""" 
Given the converged Q-matrix, this node publishes actions corresponding 
to the path most likely to lead to receiving a reward
"""

import rospy
import csv
from q_learning_project.msg import RobotMoveDBToBlock

class ExecutePath(object):

    def __init__(self):
        rospy.init_node("execute_path")
        rospy.sleep(1)

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))

        # Set up action publisher
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)

        # Initialize action
        self.action = RobotMoveDBToBlock()

        # Initialize Q matrix
        self.qmatrix = []

        # Initialize state to 0
        self.state = 0

    def read_qmatrix(self):
        with open('qmatrix.csv', newline='') as csvfile:
            csvreader = csv.reader(csvfile, delimiter=',')
            for state in csvreader:
                self.qmatrix.append([int(action) for action in state])
        return

    def move(self):
        """Find the move with largest reward; if there are multiple, choose the 
        first one. Then publish the action"""
        
        max_reward = max(self.qmatrix[self.state])
        best_action = self.qmatrix[self.state].index(max_reward)
        action = self.actions[best_action]

        # Publish the action
        self.action.robot_db = action.get("dumbbell")
        self.action.block_id = action.get("block")
        self.action_pub.publish(self.action)
        return

    def run(self):
        self.read_qmatrix()
        
        if self.qmatrix:
            self.move()

if __name__ == "__main__":
    node = ExecutePath()