#!/usr/bin/env python
"""
.. module:: action_client
   :platform: Unix
   :synopsis: action_client node for the RT1_Second_Assignment project

.. moduleauthor:: Valentina Condorelli

Description:
    Python node that implements an action client that communicates with the
    provided action server to move the robot towards a user-defined point.
    To this purpose, the `bug_0 algorithm <https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10101325>`_ was implemented.

Subscribers:
    ``/pos_and_vel`` -> custom message to obtain and print the robot position, linear velocity along x-axis and angular velocity around z-axis


Publishers:
   ``/odom`` -> robot's current position, velocity and other odometry data
   ``/reaching_goal/result`` -> robot's current status

Action client topic:
   ``/reaching_goal`` -> used to communicate with the action server "bug_as"
   
**Functions:**
"""

import rospy

# import the needed messages
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import actionlib_msgs.msg
# used for the custom message
import rt1_second_assignment.msg

# import the needed services
from std_srvs.srv import *

# import sys to output eventual errors in sys.stderr
import sys
# import select to perform a non-blocking input
import select

# import the os so that all the nodes can be killed if the user does not want to create another target
import os

# import custom message
from rt1_second_assignment.msg import pos_and_vel

# import custom services
from rt1_second_assignment.srv import Last_input_coordinates, Last_input_coordinatesRequest, Last_input_coordinatesResponse
from rt1_second_assignment.srv import Average_pos_vel, Average_pos_velRequest, Average_pos_velResponse

# declare a variable to know if the goal point has been reached or not
goal_has_been_reached = False


def status0():
    """ First status of the dictionary used to implement a switch-case structure in the ``robot_status()`` function
    
    :param goal_input: user input to set the goal
    :type goal_input: string
    :param x: x coordinate of the goal, taken from ``goal_input``
    :type x: float
    :param y: y coordinate of the goal, taken from ``goal_input``
    :type y: float
    
    """
    global tmp_status
    """ This variable is used to switch between the different robot statuses
    """

    # waiting for the user input to define the goal
    goal_input = input(
        "\n\n Please enter 2 numbers for the x and y value of the goal to reach, separated by a comma:  ")

    # error checking in case of letters present in the input string
    try:
        # get the x and y values from the input string by:
        # removing any space if present
        goal_input = goal_input.replace(" ", "")

        # splitting the message based on the comma
        new_goal = goal_input.split(",")

        # error checking in case only one number was inserted
        if len(new_goal) >= 2:
            # saving x and y in two new variables
            x = float(new_goal[0])
            y = float(new_goal[1])

            # use x and y to define the planning goal to communicate from action client to action server
            planning_goal = rt1_second_assignment.msg.PlanningGoal()
            planning_goal.target_pose.pose.position.x = x
            planning_goal.target_pose.pose.position.y = y

            # send the goal to the action server
            client.send_goal(planning_goal)

            # Go to the second state, since now the robot should be moving
            tmp_status = 1
        else:
            print("\nWrong input.")

    except ValueError:
        print("\nWrong input.")


def status1():
    """ Second status of the dictionary used to implement a switch-case structure in the ``robot_status()`` function
    
    :param cancel_input: user input to, eventually, cancdel the goal
    :type cancel_input: string
    """
    global tmp_status
    global goal_has_been_reached
    """ This variable is used to check whether the goal has been reached by the robot
    """
    global printed_status1

    # the custom service is executed everytime the user enters a new goal/target
    exec_custom_service()

    # exec the service related to the custom msg
    exec_custom_msg_service()

    # wait for the user to eventually press 'q' to cancel the goal
    if printed_status1 == False:
        print("\n\n The robot is moving. If you want to cancel the current goal, please enter 'q'  ")
        printed_status1 = True

    # with a simple input, we program was infinitely waiting for a user input, even when the target was reached
    # the select is a non-blocking version of "input()" when put in reading mode as below
    cancel_input = select.select([sys.stdin], [], [], 1)[0]

    # if there is an input
    if cancel_input:
        # get the input character
        input_character = sys.stdin.readline().rstrip()
        if input_character == "q":
            # Cancel the goal and go to the third state
            client.cancel_goal()
            tmp_status = 2
        else:
            print(
                "\n Wrong input. If you want to cancel the current goal, please enter 'q'  ")

    # if the goal has been reached (so status dwitched to 3 in the callback), still go to status2
    # as the distinction is made there
    if goal_has_been_reached:
        tmp_status = 2


def status2():
    """ Third status of the dictionary used to implement a switch-case structure in the ``robot_status()`` function
    
    :param new_goal_input: user input to set a new goal or exit the program
    :type new_goal_input: string
    """
    global tmp_status
    global goal_has_been_reached
    global printed_status1

    # distinction between status == 2 (goal cancelled) and status == 3 (goal reached)
    # made with the variable goal_has_been_reached
    if goal_has_been_reached:
        # create and update a new variable called "last_input" for the custom service
        new_goal_input = input(
            "\n\n Yay! The goal has been reached!\n Do you want set a new goal? Type 'y' for yes, 'n' for no:  ")
    else:
        new_goal_input = input(
            "\n\n The goal has been cancelled successfully.\n Do you want set a new goal? Type 'y' for yes, 'n' for no:  ")

    # delete eventual spaces
    new_goal_input = new_goal_input.replace(" ", "")

    # if the user wants to set a new goal
    if new_goal_input == "y":
        # reset go back to status0
        tmp_status = 0
        goal_has_been_reached = False
        printed_status1 = False
    elif new_goal_input == "n":
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n", "")

        for node in nodes:
            os.system("rosnode kill " + node)
    else:
        # new_goal_input = input("\n\n Wrong input.\n Do you want set a new goal? Type 'y' for yes, 'n' for no:  ")
        print("\n\n Wrong input")


def default():
    """ Default status of the dictionary used to implement a switch-case structure in the ``robot_status()`` function. It corresponds to an error case.
    """
    print("\n\n Error in tmp_status value\n")


def switch(case):
  """ Dictionary used to implement a switch-case structure in the ``robot_status()`` function
  """
  switcher_dictionary = {
       0: status0,
       1: status1,
       2: status2
       }

  switcher_dictionary.get(case, default)()

def publish_custom_message(msg):
    """
    This function publishes the custom-defined message *pos_and_vel*

    **Args**:
    
    * msg(Odometry): the robot's position and vellocity
    """
    # get position, linear velocity and angular velocity from the msg in the /odom topic
    position = msg.pose.pose.position
    vel_lin = msg.twist.twist.linear
    vel_ang = msg.twist.twist.angular

    # define the custom message
    pos_vel = pos_and_vel()
    pos_vel.x = position.x
    pos_vel.y = position.y
    pos_vel.vel_x = vel_lin.x
    pos_vel.vel_z = vel_ang.z

    # publish the custom message
    publisher.publish(pos_vel)


def callback_goal_result(msg):

    """
    Callback function used to check if the goal has been reached by the robot

    **Args**:
    
    * msg(PlanningActionResult): the robot's status
    """
    global goal_has_been_reached

    # get the status of the action server from the msg in the /reaching_goal topic
    status = msg.status.status

    if status == 3:
        goal_has_been_reached = True


def exec_custom_service():
   """
   This function executes the custom-defined service when called
   """

   # indefinitely wait for the service until it starts to avoid the client to start
   # before the server
   rospy.wait_for_service('last_input_coordinates')

   try:
        # handler for the service
        custom_service = rospy.ServiceProxy(
            'last_input_coordinates', Last_input_coordinates)

        # define the request and the response of the service
        request = Last_input_coordinatesRequest()
        response = custom_service(request)

   # error handling
   except rospy.ServiceException as e:
        print("\n Service call failed: %s" % e)


def exec_custom_msg_service():
   """
   When called, this function executes the custom-defined service which returns information based on the custom-defined message
   """

   # indefinitely wait for the service until it starts to avoid the client to start
   # before the server
   rospy.wait_for_service('average_pos_vel')

   try:
        # handler for the service
        custom_msg_service = rospy.ServiceProxy(
            'average_pos_vel', Average_pos_vel)

        # define the request and the response of the service
        request = Average_pos_velRequest()
        response = custom_msg_service(request)

   # error handling
   except rospy.ServiceException as e:
        print("\n Service call failed: %s" % e)


def robot_status(): 
    """
    This function manages all the robot's actions in the ``main()`` function
    
    :param tmp_status: global variable used in the state machine to switch between the states
    :type tmp_status: int
    """

    #initially, the status is 0, as the robot waits for an input when the program starts
    global tmp_status
    tmp_status = 0
    global goal_has_been_reached
    global printed_status1
    printed_status1 = False

    while not rospy.is_shutdown():
        switch(tmp_status)

def main():
    """
    This is the main function of the project, where all the necessary setups are done
    """
    try:
        # initialize the node
        rospy.init_node('action_client')

        # inizialize the publisher of the custom message (global so that it can be seen inside the various functions
        global publisher
        publisher = rospy.Publisher("/pos_and_vel", pos_and_vel, queue_size=1)
        """ 
        Publisher of the custom-defined message, which returns the robot's position and velocity on specific axes
	"""

        # inizialize the subscriber to the /odom topic so that the necessary values for the custom message can be retrieved
        subscriber = rospy.Subscriber(
            "/odom", Odometry, publish_custom_message)
        """ 
        Subscriber to the robot's odometry
	"""

        # initialize the subscriber to the /reaching_goal/result topic to know the status of the action server
        sub_result = rospy.Subscriber(
            '/reaching_goal/result', rt1_second_assignment.msg.PlanningActionResult, callback_goal_result)
        """ 
        Subscriber to the robot's result, so the status of the action server
	"""

        # finally, initialize the action client
        global client
        client = actionlib.SimpleActionClient(
            '/reaching_goal', rt1_second_assignment.msg.PlanningAction)
        """ Action client initializer
		"""

        # indefinitely wait for the action server until it starts to avoid the client to start
        # before the server
        client.wait_for_server()

        # Call the function that manages all the robot's action
        robot_status()

    except rospy.ROSInterruptException:
        print("\n Error: program died unexpectedly", file=sys.stderr)


if __name__ == '__main__':
    main()
    
