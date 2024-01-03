# RT1_Second_Assignment
 Second assignment of the Research Track 1 course of the Robotics Engineering master, at UniGe. 
 Made by Valentina Condorelli, S4945679.

# Second assignment
## Introduction

The second assignment of Research Track 1 consists of writing three Python nodes to control a robot in a 3D environment and achieve different tasks. More to add
### How to run the program
Firstly, if you have not already installed xterm, type in the terminal:
```bash
  $ sudo apt install xterm
  ```
Then, to run the program, you can choose between two methods.
If you have git:
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


If you don't have git:
- Download the .zip folder of this project by clicking on *<> Code* -> *Download ZIP*
- Extract the .zip folder
- Move into the `robot-sim` folder
- Follow the previous procedure starting from step 3
