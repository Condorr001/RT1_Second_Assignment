
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
- If it still does not work, please close the current terminal and open a new one


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
  ![alt text](https://github.com/Condorr001/RT1_Second_Assignment/blob/main/flowchart/actionclient_flowchart.jpg)
  Note: click [here](https://viewer.diagrams.net/?tags=%7B%7D&target=blank&highlight=0000ff&edit=_blank&layers=1&nav=1&title=actionclient_flowchart.drawio#R7R1bl5s2%2Btf45MlzEHcek5mkzTbN2e1km83THGzLNhuMXMAz4%2F76lUDCIAmssc3FePvQDDII8d303TUx7jevv8T%2Bdv07WsBwomuL14nxMNF1ADQL%2F0NG9vmIbdCBVRws6E2Hgcfgb0gHNTq6CxYwqdyYIhSmwbY6OEdRBOdpZcyPY%2FRSvW2Jwupbt%2F4KCgOPcz8UR78Hi3Sdj7q6cxj%2FFQarNXszsL38l43PbqZfkqz9BXopDRkfJ8Z9jFCa%2F7V5vYchAR6Dy%2FfP%2B%2B%2Fhl5%2F2L%2F%2F4V%2FKX%2F%2B8Pv337%2Buc0n%2BzTWx4pPiGGUXry1A%2Fz1Z9fF9M%2FNvut8%2FeXr%2BaT982cenTuZz%2FcUYDRj033DIJwgQFKL1GcrtEKRX748TD6IUa7aAHJezR8dbjnC0JbPAjw4H9hmu4pdfi7FOGhdboJ6a%2F5O8mLOKQd%2BWJ6X4J28Rw2fCYjPD9ewbQJHAe8YoaAaAPTeI8fjGHop8FzdXU%2BpcxVcd8B%2BvgPioA3IEOGCzvE6%2F2wRBgMZaTYf%2B0Q%2B2GaZGB9j28A1vY1Ayf7Hf%2B1Iv8%2B4k9P2Wx4cfmE%2BW8iwsMQcydB7Ms6SOHj1s%2BA%2B4IFhAxtzzBO4esJiBPhzGaxKMu9HDgWMIGyLnGrrbWECusm2EJXZAuzT67QBVR8joI08EMCNbz4Nfl%2FhPctAUNV%2BPdFyyalJbp%2F6pqEtnUJbZtt0bZ9GzLfVCRuu1eZb9bK%2FEXwLBX5onBnTyRbP5I%2BMvPnP1cZwqZzFKI43yyCnI2kUz7AZRDlm0qxZ%2BTTV1%2BJh7N1Vkfrlo5ZK51ixl1F%2BQpCuOzsi6aEtnazMEjWMMZ%2Fo2UhPOa7JEUbovLBJCHq5Ji%2BOdnNknkczLKPxjxIRM0ntMi%2BN0XbYJ4p1XEDMLCkwoKKPMfmp3B8yu99OoCteD9hFDyXH4Zk1eQVu2ieBigaOWxzGOKviKE%2FXwfR6mmF8ATkOtmFaQnm2e1EMrGHktRPd0mVMmM0wy%2FV72VIYMDN3vDE5udRQJE6ckTkX%2BnTD9PmYQCj9I1feI7yQD%2F9YY5fi4mhPW2CKQpMmXBEZcJwJMpEoT1f3mgxbkKbcBW1CaD3akGK6sQPmEjx8cWfwfAIFRP6DbDseE9%2F2ASLRY4uiG1Of5bNRxC2RUGUZh9jfZhYD00MQP089OFJQdFlTDUQWi1nTLU7Bxj0JcrQptP9k6y%2FpLPbFS4zqs%2Bj5TIhcpvDVbGk09HnDlMZxP%2F6GyLxollC%2Fvm1tLMUMjeB8XO2DybE1UBY%2BdOZAniNNjO8IQ5H%2BAK9aspZhsSUsyXS12lN%2BGoKwjdavCd%2BVAKh0E8SrH1UwFcVvfA1SP9DfyF%2F%2FyDjdxa9engt3fawpxedyF4RKyWoWxKgs7EzZYHOXKQM6Twy8%2BXTp8r%2BV34ijaMefkvO9xZhIow8f1%2B6jcra%2BgW78vcc6Cyf8aKSC4g%2Bha9IIEWiJra35RClIHeFZlMnaYx%2BwvtcxD1EiJizH5ZBGHJDralqLkc4uitIC1NCt0ZrwsK%2BCU0NqHo1gdWrqib6NQWnyyy%2BGgNss30qDNhsPuLmvK9aoWwSfA03WxT7Gbgb7N4RWajbGIsruKBAAgWUPvlhAnNI5V4YtEu3u5Q9EUSrXGfGwizs0KBtSyhyznDblNivMmd4a4EeINO5RygULVWh6PQqFMW42%2BNLkM6J5yuzMipyJr9rkHQOWH4BJXRHl5gKli0SemtRH13FVBgBoTuqhO71SegM%2BCVsfIslxHyVnhq9JqJGp9fuNM3SK%2BxBr0411egszH%2FSvqPGkIXtbpiZjBrfXDfMZIi7BlWrxsBNRk32xcHvaepedbeZGufxE1u5W1XWuuIuRnYlfAr2BTMxTnUatrbxm9UIjeNKNn6jSw1XvxFZ5SnKKr1XWcWWWcLG%2B%2BRnYfjukjyWnEdtEYoXQeSnkDOP6Tcy4%2FoFZQZobjYuQ%2BQTkfQ%2Bo0l8S7TIqHGQarJhV%2B1BKbd0ag%2FqIn7GyC0spnWcW2qc7x3t7KLRMpp4pl5jgBRasuVx3HGmlswUBi662Z3WrIupAp8P3sElijc%2BWafo%2FSyJPibmkjRmvrE4JnUDsudy2bdAmfwM0hxbqR9EhdSMdpsZjJMsWDk0VcLiAkaOLRGO0mQPrTXpqBJCOCneeIgx%2Fij9Io83YqDG%2B9JD5PJH%2BbfDY9lVV3FKZaFa53voJqJpWVW6cnmfk2pE0%2BQJlA9dXSiiaXma9D2169Ll62o1AqqLRoskAnqdO5XdKKewBarpGrepnJmIw6bxutqbDNEhJ0Xetet9qnlsRs3e1JF7VIzKPMJsM8%2B1gNcpWUueWltOvBWykYZo9Di2WRVPnmRf1xxR%2FrcWG2Bpc00OF10D1wFOV1Y8JgNna4kWhrgV9CFLhpjKZaiqSEbNnnPhbC5PMZvrYtuMKNjGE%2FcxamLHZT2B%2Bb8uatJWcdqdQeuI7okxKg3KTGv3GlM1RPeCPGwg2cj6tvX5sIEnSYzpNmxg3ght26obUr%2B0LTpePr7C%2BS4lWu8MZaVh1I0WxEnm6SK1eUwVDuaZdlzSoBOIxTy5JEmweYHZOsjybHYJLBWOCdWAHAn0FThwq%2FamlF9k2bXt8ctt1EEZqmE2U%2B%2BTX2yRX8ajZ5nNlVBYzbLMKntcyBsDuEfa06zsYZhQw%2BGmc82h87hJtFpGlGBjN5st2p0JXC6%2F5kJlhlZX7MTorIRAeUJ6ph%2FLcdu7jmxZCnu%2BrPyuvT3%2FNtoEmbqqlOq1URBb5pHUmsxbvN%2BSoXd%2FvSOT0uhyFoHzo5TdNPejedYML38%2B9zYPQ%2F11TM6N1b%2B5eBsbtqnaVajfDdsUc%2F5qCpCqld14VKomD4TMgaZK562FSszbyA8zVd0iZq%2F5YZZo5o0mP8w8lh%2BmcyF3SojnpodVHfRFnWr7SqopIpOlh8WZ%2B2pSSvsSk72yrK48PWyQ2VwOL8tk3SM63bOt23BZmao5D1avLitL1KBGI8usYx4rGzjVnK2rSyAyRR%2FJ5yLt1KfGxhAFk6kimNxO5dLlu9rUZIyCdjNGT7YSOkoEBTzmPQ6lqpmgNk9DQr%2B5S6WC8ksG1e424soMu%2FGBdpJBLaV2ONcpypuLe0mSh8cqDM71j%2FLk2ZUst27Dr8eKsI8rRr369dgyS9i4p6453gYYnqeOD1RLXRgyp3VrHgxL9A1Jkj1Fgu8JgDqnGGIAWgIAPQkAW0vvtFRaxlygCIblbfaTqVnny%2B9GNSkKQYttWz9RNSk6rxQz8UrOhVQTV%2BM1Da8DTcMWuXmUW5VqoNzutbrbFhWH8aSd2M2NKEjaiV7hgAu1oTCqkzrV51vUAkVXf9Zqfe0nTzMIo6esxzvmmlLoXIrt3k38otVoIZokpaR6pwfs2KJSN0a5xeB3VG71GkaxRYfLgvSiDGg1VVZ%2FLT1vIGszUT7FQNq6kvZDZCpm6YwCvufjnXSKz8tJqU9kzmxGnrXaxJF17DiIqqWqVmsCCUN2mstiK2TPD9ksAJKjELo9V8m5EZF2biJ8zS7PG8oG38WgRk%2B%2FWDGPyADj0d1y2mzS3Vyt6m3Tz9PdOq%2FFAtptZGbYqpYQ0HrVKYDkbIIRJQ1Tcmvyg1eNIfM8fuJc4h1k4IvsJK%2BmE7msd3uHr6YDsnOSus21cG4jP9JR7TPt9Jof6Yy5DNs5WoateybnVLm6AiF2uMb%2F%2BYnCoyYs%2BzaMvtXnbbpVOgKGfaQ105EH2nGSu6JlNiJVxG3eDLU7y%2FWqnXwvlCrKUkO7TxV1RD%2F7lbpmDT7QJVNViqOjOlFV3NsIKTmqKe5uryElR1LJnD%2FGdQz2D05Y2hhrnbUHJvxA4LGbz2GSLHdhmC%2BeMsgw3Hm2oaKxd9oJ2BXt10sAPq8qCwcMekPWZ7Rb0N%2BGseSq9tVxezWWXNExeqyuct9UV5lAlhIdwZeCaUgw511Ufm5CWvdG79KhMIqrDc6r4KokKr4pFeksmu8tW8jiRdipicxC3pFqFOKtlpP4oi6yhdzb8JG7yqpdry5yr4fqRSA1SjfoufR7jFI%2FLV0vYAjL13ARlC9DNP9ZUEPVim0Ux6cbvMdd74Z2WYu3MHE7yz7ybuNgQmVW9Xo9mNATBedYCjq8Ghl44CZgsfOlrqbhjVtfSzw5HCLhU111iGV5grZpqtYLW61pm6JI%2BgNSbT7rJYjJF%2FNxdgpRnHUNzKOFGjGG9yM613hE5xI3589RL%2FmIvvfIOcznHLDck6AoWlgXgkIXC2SKs%2FI6MUs90XVWPtucaznDywQlkuTF9RByOgHgS5UkEltWSd2axPYu7h84pVRpqOf1MDI9fgiaHO3dODcKTY5R1am%2BDV0QFN0mWAKWYThyk8ZTPoJaOzdof16GHkvdGmPHEUpsTWaNZbPDHi9bpz7tzszxxEBJnZkTXYmZY%2Bl9mzlAE43HMcqp88VPHTvwEUXL5Dii9Z1GVEB%2FC7Ln%2FFIEN0ILibgbihpvyex9WRi2tUJ3oKk0CBun%2BtiXtufynXQsR7vTSv%2Fp1RlVlT9hXr153ksVxfOvtbXmBEGg87nbb3%2BCDajDovpAO7E3oIl5VWPxIVNJ0aRteQ47%2B%2BpqnMjFqXHlU1eIo5NHWRgG2wT26XgQdlyJD0jmeWitGBKwZNDRK1Gq3SUBqEFiV9ae6L1%2FgEufVHEPzioQCnjs3hucA3AbabGAWbMKBN1rYmyx0LJ0xpponFm%2BEy52k98kEPoQ3MaGzutIshNMWePxjqS3qPkPcucTOpvIYHehjQ9fxogElA46Bv7O9e%2FYiiR3%2FA8%3D) to see the flowchart in a better resolution.
 - `custom_service.py`: service node that, when called, retrieves, from the `/odom` topic, the last target set in input by the user, prints them and returns them. Given the structure of the `/odom` topic, the coordinates of the last target are retrieved as follows:
	 ```
		x = msg.goal.target_pose.pose.position.x
		y = msg.goal.target_pose.pose.position.y
	 ```
	 The call to this custom service is automatic: everytime the `action_client.py` switches to *status1*, it means that a new target was successfully taken as input. Therefore, the call to the custom service is made at the beginning of *status1*, since it's meaningful to know the last target set only one it changes, given that the list is always accessible in the output window.
	 
- `custom_msg_subscriber.py`: service node that subscribes to the custom message published by the `action_client.py` node. It then uses the data in the message to compute, print and return the distance between the robot and the target and the average speed of the robot, both linear along the x-axis and angular along the z-axis. For the last task, a parameter is set in the launchfile `assignment1.launch`: *window_size* = 10.0. With this parameter, the average speed is returned as the mean of the last, in this case, 10 available values of the velocity.
Since this is a service nodes, it needs to be called by the (main) `action_client.py` node in order to performe its task. In this case, the call is made periodically as long as the `action_client.py` is in *status1*, so as long as the robot is moving and, therefore, changing its position and velocity. The frequency of the call is the same the the frequency of `ros::spin()`.

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
