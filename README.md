# Q Learning Project
## Team Members:
- Tianle Liu
- David Wu 

## Implementation Plan
- Q-learning algorithm
	- Executing the Q-learning algorithm
		- Implement the Q-learning algorithm based on lecture.
		- Test: print out the selected actions and Q-matrix for the first three steps and check if they matche our hand-calculations. 
	- Determining when the Q-matrix has converged
		- Let delta be the difference between the old value (quality) and the new value we get from the equation. Once delta is within a certain epsilon value we fix, then we say that entry converges. We say the matrix converges when the current step we are at converges for the last 5 steps. 
		- Test: print out differences and see if they go to zero. 
	- Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
		- Find action with largest q value from the current state to the next state.
		- Test: print out the first 5 actions and see if they match our expectations calculated by hand using the Q-matrix. 
	        
- Robot perception
	- Determining the identities and locations of the three colored dumbbells 
		- Use RGB camera feed to get an image of the dumbell setup. With this, we will have a matrix telling us which colors are where with respect to our robot camera. Thus, we can identify the different dumbell colors by determining the center of the red, green, and blue pixels respectively. We will also use the scan to determine how far we are from the dumbells.
		- Test: Print out the x locations of the center color pixels and check if it matches the Rviz view. Print out scan and RBG camera messages. 
	- Determining the identities and locations of the three numbered blocks
		- Use `keras_ocr` to determine which block has which character. Use scan to determine how far we are from a block.
		- Test: print out numbers and compare against visual cues 
          
- Robot manipulation & movement
	-   Picking up and putting down the dumbbells with the OpenMANIPULATOR arm
		- We will use the Moveit topic to control the arm. To determine angles of how to pick up the arm, we will use the GUI to help us determine angles etc.
		- We will test using the GUI to determine the correct angle, and by running the script and observing
	-   Navigating to the appropriate locations to pick up and put down the dumbbells
		- We will use an approach similar to line follower. We determine which dumbell to navigate to by trying to center its pixels. We will also move until the scan topic tells us we are close. Navigating to the block is a similar strategy: trying to center the robot with respect to the block. We will use proportional control like in line follower to execute this.
		- We will test this by manually see what the robot is doing by running script and printing out action and seeing if it matches the behavior.

### Timeline
- May 3: Finish implemeting and testing Q-Learning part
- May 6: Finish implemeting and testing perception part 
- May 9: Finish implemeting and testing movement part
- May 11:  Finishing touches, fix bugs, recordings, clean code

## Writeup
### Gif 
![drive_square gif](https://github.com/Tianlel/q_learning_project/blob/master/recordings/q_learning.gif)
(5x speed)
### Objectives description
TODO
### High-level description
TODO
### Q-Learning Algorithm Description
TODO
### Robot Perception Description
### Robot Manipulation and Movement
### Challenges
### Future Work
### Takeaways
