#!/usr/bin/env python
"""
.. module:: custom_service
   :platform: Unix
   :synopsis: custom_service node for the RT1_Second_Assignment project

.. moduleauthor:: Valentina Condorelli

**Description**:

    Python node that implements the custom-defined service used to return the last target coordinates sent in input by the user.


Publishers: 
    ``/reaching_goal/goal`` -> robot's current goal

"""

import rospy
import rt1_second_assignment.msg

from rt1_second_assignment.srv import Last_input_coordinates, Last_input_coordinatesResponse

# Callback for result subscriber
def srv_callback(msg):
	"""
	Callback function used to get the current goal coordinates
	
	**Args**:
	
	* msg(PlanningActionGoal): robot's current goal
	
	:param x: current goal x coordinate
	:type x: float
	:param y: current goal y coordinate
	:type y: float
	"""
	global x
	global y
	global printed_service
	printed_service = False
	
	# Get the status of the result
	x = msg.goal.target_pose.pose.position.x
	y = msg.goal.target_pose.pose.position.y

def srv_function(required):
	"""
	This function returns the coordinates of the current robot's goal
	
	**Args**:
	
	* required: required value for the service function to correctly operate
	
	:return: service response, implementing ``x`` and ``y``
	"""
	global x
	global y
	global printed_service
	
	# print the coordinates of the last input target
	if not printed_service:
		print(f"\n\n Last target input x coordinate: {x} m")
		print(f" \n Last target input y coordinate: {y} m")
		printed_service = True
		
	return Last_input_coordinatesResponse(x, y)

if __name__ == '__main__':
	try:
		# initialize the node
		rospy.init_node('custom_service')
		
		# initialize the custom service
		srv = s = rospy.Service('last_input_coordinates', Last_input_coordinates, srv_function)
		"""
		Custom service initialization
		"""
		
		# subscribe to the /reaching_goal/goal topic to retrieve the necessary values
		sub_result = rospy.Subscriber('/reaching_goal/goal', rt1_second_assignment.msg.PlanningActionGoal, srv_callback)
		"""
		Subscriber to the robot's current goal
		"""
		
		# execute in a loop
		rospy.spin()
		
	except rospy.ROSInterruptException:
		print("\n Error: program died unexpectedly", file=sys.stderr)
