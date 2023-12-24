#!/usr/bin/env python

import rospy
import rt1_second_assignment.msg

from rt1_second_assignment.srv import Last_input_coordinates, Last_input_coordinatesResponse


# Callback for result subscriber
def srv_callback(msg):
	global x
	global y
	
	# Get the status of the result
	x = msg.goal.target_pose.pose.position.x
	y = msg.goal.target_pose.pose.position.y
		
# function for the custom service
def srv_function(req):
	global x
	global y
	
	# print the coordinates of the last input target
	print(f"\n\n Last target input x coordinate: {x} m")
	print(f" \n Last target input y coordinate: {y} m")
	return Last_input_coordinatesResponse(x, y)

if __name__ == '__main__':
	try:
		# initialize the node
		rospy.init_node('custom_service')
		
		# initialize the custom service
		srv = s = rospy.Service('last_input_coordinates', Last_input_coordinates, srv_function)
		
		# subscribe to the result of the /reaching_goal topic to retrieve the necessary values
		sub_result = rospy.Subscriber('/reaching_goal/goal', rt1_second_assignment.msg.PlanningActionGoal, srv_callback)
		
		# execute in a loop
		rospy.spin()
		
	except rospy.ROSInterruptException:
		print("\n Program interrupted before completion", file=sys.stderr)
