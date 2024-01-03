
# RT1_Second_Assignment
 Second assignment of the Research Track 1 course of the Robotics Engineering master, at UniGe. 
 Made by Valentina Condorelli, S4945679.

# Simulator and graphics

For the graphic part of this project, the Gazebo simulator was used to test the robot behaviour in a 3D environment. All the files used to spawn the robot and its surrounding environment were provided by our professor in a base repo, which you can find at the following link: https://github.com/CarmineD8/assignment_2_2023.git

# Second assignment
## Introduction

The second assignment of Research Track 1 consists of implementing three ROS nodes to control a 4-wheeled robot in a 3D environment and achieve different tasks. Particularly, the is to create a main node with which the user can set the target coordinates they want the robot to reach and cancel them if necessary. The other two nodes are responsible of providing useful services, such as printing the last coordinates set by the user and the robot position and average speed periodically.
### How to run the program
Firstly, if you have not already installed xterm, type in the terminal:
```bash
  $ sudo apt install xterm
  ```
Then, to run the program, you can choose between two methods.
#### If you have git
- Move into the `src` folder of your ROS workspace
- Clone this repo by typing in the terminal:
  ```bash
  $ git clone https://github.com/Condorr001/RT1_Second_Assignment.git
  ```
- Move into the `scripts` folder
- To make the Python nodes executable, type in the terminal:
  ```bash
  $ chmod +x *.+py
  ```
- Finally, type in the terminal:
  ```bash
  $ roslaunch rt1_second_assignment assignment1.launch
  ```
If the last command does not work, please:
- Go to the main folder of your ROS workspace
- Type in the terminal:
  ```bash
  $ rospack list
  ```
- Run again the `roslaunch` command


#### If you don't have git
- Download the .zip folder of this project by clicking on *<> Code* -> *Download ZIP*
- Extract the .zip folder
- Move into the `robot-sim` folder
- Follow the previous procedure starting from step 3

## The setup
After the program is launched, you should see four different elements:
- A 3D environment with some walls and the robot positioned at the center
- An xterm window for the `action_client.py` node, waiting for your input
- An xterm window for the `custom_service.py` node, empty because this service is called every time new coordinates are inserted
- An xterm window for the `custom_msg_subscriber.py` node, empty because this service is called periodically while the robot is moving

If you can not see the three xterm windows, please check the opened windows: they usually fall under the Gazebo window as it spawns later.

## The nodes
All the nodes for this project are implemented in Python and can be found in the `scripts`   folder. There are six nodes in total, which can be splitted into two groups.

### Nodes provided
The three nodes that were provided in the base repo are:
-  `go_to_point_service.py`: service node responsible for controlling the robot so that it moves forward to the target point. It's implemented as a three-state machine (*Fix Heading*, *Go Straight* and *Done*);
- `wall_follow_service.py` : service node responsible for the robot behaviour when a robot is detected as an obstacle for reaching the target point. It's implemented with three functions which make the robot turn left and follow the robot until its end
- `bug_as.py` : node that merges the two previous nodes by applying the **bug0 algorithm**. It is implemented as an action server, so that an action client could be built to communicate with it

### Nodes implemented
The implementation of the other three nodes was the main focus of the project. These are:
- `action_client.py` : action client node responsible for the communication with the action server. Moreover, this node publishes the robot position, linear velocity on x-axis and angular velocity around z-axis as a custom message and subscribes to the `/reaching_goal/result` topic to get the status of the robot. Particularly, even though the status can have different values, only four are considered:
	- 0: the robot is not moving and is waiting for a target point to reach
	- 1: the robot is moving towards the target point
	- 2: the goal has been cancelled, so the robot has stopped moving
	- 3: the target has been successfully reached

  The behaviour of this node is explained in the flowchart below:
 - `custom_service.py`: service node that, when called, retrieves, from the `/odom` topic, the last target set in input by the user, prints them and returns them. Given the structure of the `/odom` topic, the coordinates of the last target are retrieved as follows:
	 ```
		x = msg.goal.target_pose.pose.position.x
		y = msg.goal.target_pose.pose.position.y
	 ```
	 The call to this custom service is automatic: everytime the `action_client.py` switches to *status1*, it means that a new target was successfully taken as input. Therefore, the call to the custom service is made at the beginning of *status1*, since it's meaningful to know the last target set only one it changes, given that the list is always accessible in the output window.
	 
- `custom_msg_subscriber.py`: service node that subscribes to the custom message published by the `action_client.py` node. It then uses the data in the message to compute, print and return the distance between the robot and the target and the average speed of the robot, both linear along the x-axis and angular along the z-axis. For the last task, a parameter is set in the launchfile `assignment1.launch`: *window_size* = 10.0. With this parameter, the average speed is returned as the mean of the last, in this case, 10 available values of the velocity.
Since this is a service nodes, it needs to be called by the (main) `action_client.py` node in order to performe its task. In this case, the call is made periodically as long as the `action_client.py` is in *status1*, so as long as the robot is moving and, therefore, changing its position and velocity. The frequency of the call is set with a parameter in the launchfile `assignment1.launch`, called *freq*.

## Messages and services
Along with the implemented nodes, custom messages and services were introduces. 

Firstly, in the `msg` folder, the custom message `pos_and_vel.msg` can be found. Its structure is straightforward: 
- two float64, *x* and *y*, for the position of the robot;
- other two float64, *vel_x* and *vel_z*, respectively for the robot linear velocity along the x-axis and angular velocity around the z-axis. 

The type float64 has been chosen because the node `action_client.py` is the publisher of this message and retrieves the data from the `/odom` topic, in which all these values are of float64 type.
The subscriber for this message is the `custom_msg_subscriber.py` node, which uses its data as previously described.

Secondly, in the `srv` folder, two custom services can be found: `Last_input_coordinates.srv` and `Average_pos_vel.srv`.
`Last_input_coordinates.srv` is the service correspondent to the `custom_service.py` node. Indeed, its structure is defined by:
- int32 input: the request value, sent by `action_client.py`to `custom_service.py` to call the service;
- float64 x and float64 y: the coordinates of the last target set by the user, printed and returned by `custom_service.py` as the response.

On the other hand, `Average_pos_vel.srv` is the service correspondent to the `custom_msg_subscriber.py` node. Indeed, its structure is defined by:
- int32 input: the request value, sent by `action_client.py`to `custom_service.py` to call the service;
- float64 dist, float64 lin_vel and float64 ang_vel: respectively, the distance between the robot and the target, the robot average linear velocity on x-axis and its average angular velocity around z-axis, as response values printed and returned by `custom_msg_subscriber.py`

## Further improvements
This project can be improved in a few ways.
First of all, the way the robot moves is not optimal: slowness aside, since it always turns left when it faces a wall, it can happen that it takes a longer way to reach a near target point. 
Moreover, if it gets stuck in the corner of a wall while moving, not much can be done to free it, apart from restarting the program.
Finally, the robot is not aware of the map bounds: if a target outside the field is given, the robot will endlessly follow the outer walls trying to reach that target. If the map bounds are known, a solution could be to limit the user input and not accept targets outside the bounds. 
