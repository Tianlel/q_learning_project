#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv
import time
from random import choice, randint
from q_learning_project.msg import QMatrix, QMatrixRow
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveDBToBlock

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"
t0 = time.time()

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
        self.qmatrix = QMatrix()
        self.qmatrix.q_matrix = [QMatrixRow(q_matrix_row = [0 for action in range(9)]) for state in range(64)]
            
        # publisher for QMatrix
        self.qmatrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)

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
        # count how long the values of Q matrix have stayed the same
        self.convergence_cnt = 0
        
        self.reward_received = True
        self.converged = False

    def perform_action(self):  
        # wait till an reward is received
        if not self.reward_received:
            rospy.sleep(1)
            return

        # randomly select a valid action 
        valid_actions = [x for x in self.action_matrix[self.state] if x > -1]

        # if no valid actions, return and wait the world to be reset
        if not valid_actions:
            # reset everything to the original state
            self.state = 0
            self.next_state = None
            self.action = None
            self.reward_received = True
            return

        action = choice(valid_actions)
            
        # update action at time t
        self.action = int(action)

        # update state at time t+1
        self.next_state = np.where(self.action_matrix[self.state] == action)[0][0]

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

        # get current Q value given state and action
        q = self.qmatrix.q_matrix[self.state].q_matrix_row[self.action]

        # find maximum q value for the next state
        max_q = max(self.qmatrix.q_matrix[self.next_state].q_matrix_row)

        # calculate new Q value
        q_new = int(q + self.alpha*(data.reward + self.gamma*max_q - q))
        self.qmatrix.q_matrix[self.state].q_matrix_row[self.action] = q_new
        #print(q + self.alpha*(data.reward + self.gamma*max_q - q_t))
        # update current state
        self.state = self.next_state

        # publish Q matrix
        self.qmatrix_pub.publish(self.qmatrix)

        # check if the Q matrix has converged 
        if q == q_new:
            self.convergence_cnt += 1
        else:
            self.convergence_cnt = 0

        if self.convergence_cnt == 50:
            print("self.convergence_cnt reached 50 at "+str(time.time()-t0))
        if self.convergence_cnt == 100:
            print("self.convergence_cnt reached 100 at "+str(time.time()-t0))
        if self.convergence_cnt == 150:
            print("self.convergence_cnt reached 150 at "+str(time.time()-t0))

        # if the Q matrix has remained the same for 200 steps, then we consider it to be converged
        if self.convergence_cnt == 200:
            print("self.convergence_cnt reached 200 at "+str(time.time()-t0))
            print("Q Matrix has converged! Training done!")
            self.converged = True
    
    """Save the q_matrix file to csv"""
    def save_q_matrix(self):
        with open('qmatrix.csv', 'w', newline='') as fp:
            writer = csv.writer(fp)
            for state in range(64):
                writer.writerow(self.qmatrix.q_matrix[state].q_matrix_row)
        return

    """Run Q-Learning algorithm until convergence"""
    def run(self):
        while self.converged is False:
            self.perform_action()
            rospy.sleep(1)

        self.save_q_matrix()
        
if __name__ == "__main__":
    node = QLearning()
    node.run()
