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

        # Set up action publisher
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)

        # Initialize action
        self.action = RobotMoveDBToBlock()

        # Initialize Q matrix
        self.qmatrix = []

    def read_qmatrix(self):
        with open('qmatrix.csv', newline='') as csvfile:
            csvreader = csv.reader(csvfile, delimiter=',')
            for state in csvreader:
                self.qmatrix.append([int(action) for action in state])

        print(self.qmatrix)
        return


    def find_next_best_move(self):
        # TODO
        return

if __name__ == "__main__":
    node = ExecutePath()
    node.read_qmatrix()
