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
#### Selecting and executing actions for the robot
TODO
#### Updating the Q-matrix
TODO
#### Determining when to stop iterating through the Q-learning algorithm
TODO
#### Executing the path most likely to recieve a reward after the Q-matrix converged 
TODO

### Robot Perception Description
For both identifying colored dumbbells and numbered blocks, we use the image received from the callback function `image_callback` and store it in `self.image` in the Action class. Here, we also convert the image to HSV format and store it in `self.hsv`.
#### Identifying colored dumbbells
Identifying colored dumbbells is done by the first half of the function `move_to_dumbell`. On input of a color red, green, or blue, we take the image from `self.image` and create a mask with an upper and lower range of HSV values. With this, the color we want (red, green, or blue) will show up in the picture while the other pixels will be black. With this, using the moments function in the `cv2` library, we can determine where the target color in sight is and identify the dumbbell. If there is no dumbbell in the image, then we turn until we find one.
#### Identifying numbered blocks
Identifying numbered blocks is done in the functions `check_for_dumbbells`, `determine_block_center`, and `move_to_block`. In `move_to_block`, we use the `keras_ocr` library to give a list of prediction box tuples of text in an image we input from `self.image`. Then, given a target block we want to go to, we choose specific prediction box tuples that will be useful for us to identify the block. First, we iterate through all the predictions and see if they match a target char in `possible_boxes`, as image recognition can be off sometimes. However, we require the first block identification to match the exact number to help with exactness and keep track of this with `self.block_visible`. This way, we do not place the dumbbell down in front of the other dumbbells and keep turning without moving forward to find the right block. We then pass in these possible boxes into `check_for_dumbells`, as we want to make sure our image prediction is not affected by dumbbell presence making text look different. This function works by checking to see if the box has any color in it with masks of the 3 dumbbell colors. With all these boxes filtered, we then use `determine_block_center` to calculate the location of the block in the image, which we then move towards.

### Robot Manipulation and Movement
#### Moving to pick up dumbbell
Above, we explained how we identified the dumbbell. Once we have the center (`cx`) located from the moments object, we use proportional control to move the dumbbell closer and closer to the dumbbell, slowing down as we get closer with information from the `self.laserscan_front` data. Once we reach a distance of `0.22` from the dumbbell, we move into the pickup state.
#### Picking up dumbbell
Picking up the dumbbell is done in the `reset_gripper` and `pick_up_gripper` functions. In `reset_gripper`, we set the arm and gripper to a state such that the robot can move the gripper on the handle of the dumbbell to pick it up. In `pick_up_gripper`, we move the arm and gripper to pick up the dumbbell, then move back away from the dumbbells, and change the state to go to the block. We found appropriate values for the arm and gripper for both these functions by playing with the GUI and testing through observation to see what worked and what didn't work.
#### Moving to numbered block with the dumbbell
Once we have identified the block through `keras_ocr`, set the `self.block_visible` variable to `True`, and selected the appropriate boxes to use, we calculate the center of all the boxes that correspond to the appropriate block number. In combination with data from `self.laserscan_front` data, we use proportional control to move the robot slowly but surely toward the block. Note, that every iteration of `move_to_block`, we move for a small amount of time, `0.75` seconds to make sure our robot does not veer off course. Sometimes, our robot may not identify the correct numbers, especially if it has turned too much and/or is too close to the block to identify correctly. To help with this, we keep track of what direction the robot has last turned. That way, if it happens that we can't identify the block, we go forward and turn the other direction in case it was the case that we turned too much. If it is the case that we were too close to the block, our next iteration will turn back the other way, zigzagging our way towards the block. Once we have reached a certain distance from the front, we move to the drop dumbbell state. 
#### Putting the dumbbell back down
This is done in the `drop_gripper` function. We set the arm and gripper to a state such that the dumbbell drops vertically and gracefully with as little shaking as possible. We then move back slowly for some time, and reset our state to STOP to continue listening for actions from the `dispatch_action` node. We found appropriate values for the arm and gripper for both these functions by playing with the GUI and testing through observation to see what worked and what didn't work.

### Challenges
TODO

### Future Work
TODO

### Takeaways
TODO
