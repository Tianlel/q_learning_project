#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv
from random import choice, randint
from q_learning_project.msg import QMatrix, QMatrixRow
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveDBToBlock

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")
        rospy.sleep(1)

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

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

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the red, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # Initialize Q matrix
        self.qmatrix = [[0 for action in range(9)] for state in range(64)]
            
        # publisher for QMatrix
        self.qmatrix_pub = rospy.Publisher("q_matrix", QMatrix, queue_size=10)

        # subscribe to reward
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.update_q_matrix)
        rospy.sleep(1)

        # publisher for robot action
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)
        rospy.sleep(1)

        # initialize RobotMoveDBToBlock message
        self.robot_action = RobotMoveDBToBlock()

        # action performed at time t
        self.action = None

        # state at time t, initialized to the orginial state
        self.state = 0
        # state at time t+1
        self.next_state = None

        # learning rate
        self.alpha = 1

        # discount factor
        self.gamma = 0.8

        self.reward_received = True
        self.converged = False

    def perform_action(self):   
        # wait till an reward is received
        if not self.reward_received:
            rospy.sleep(1)
            return

        # randomly select a valid action 
        valid_actions = [x for x in self.action_matrix[self.state] if x > -1]
        action = choice(valid_actions)
            
        # update action at time t
        self.action = int(action)

        # update state at time t+1
        self.next_state = np.where(self.action_matrix[self.state] == action)

        # perform action
        action = self.actions[self.action]
        self.robot_action.robot_db = action.get("dumbbell")
        self.robot_action.block_id = action.get("block")
        self.robot_action_pub.publish(self.robot_action)
        #print(action)

        self.reward_received = False
        return
     
    def update_q_matrix(self, data):
        self.reward_received = True

        # update Q matrix given action and reward
        q_t = self.qmatrix[self.state][self.action]

        # find maximum q value for the next state
        max_q = max(self.qmatrix[self.next_state])

        self.qmatrix[self.state][self.action] = q_t + self.alpha*(data.reward + self.gamma*max_q - q_t)

        # update current state
        self.state = self.next_state

        # publish Q matrix
        # TODO

        # check if the Q matrix has converged 
        # TODO
       
    def save_q_matrix(self):
        with open('file.csv', 'w', newline='') as fp:
            writer = csv.writer(fp)
            writer.writerows(self.qmatrix)
        return

    def run(self):
        while self.converged is False:
            self.perform_action()
            rospy.sleep(1)

        self.save_q_matrix()
        
if __name__ == "__main__":
    node = QLearning()
    node.run()
