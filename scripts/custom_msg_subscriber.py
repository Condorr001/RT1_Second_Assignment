#!/usr/bin/env python

import rospy
import math
import time

from rt1_second_assignment.msg import pos_and_vel
from rt1_second_assignment.srv import Average_pos_vel, Average_pos_velResponse

# get the window size from the parameters in the launch file
window_size = rospy.get_param('/window_size')

# initialize the global variables
dist = 0
lin_vel = 0
ang_vel = 0


def msg_callback(msg):
	global dist
	global lin_vel
	global ang_vel
	
	# get the the desired position of the robot
	x_desired = rospy.get_param("des_pos_x")
	y_desired = rospy.get_param("des_pos_y")
	
	# get the current position of the robot
	x = msg.x
	y = msg.y
	
	# compute the distance between the robot and the desired position
	"""
	N.B.: there is a base error between the robot actual position and the target.
	Indeed, the robot switches to status 3 (equivalent to target reached) when the 
	distance to the target is still 0.50.
	Therefore, to have a better output in the message, 0.50 is subtracted from the 
	computed distance, with the minimum set to 0.0 as a distance can not be negative 
	"""
	
	dist = math.dist([x_desired, y_desired], [x, y]) - 0.50
	if dist < 0.00:
		dist = 0.00
	
	# save the current linear and angular velocities of the robot
	vel_x_linear = msg.vel_x
	vel_z_angular = msg.vel_z
	
	# compute the average linear velocity based on the window size
	if isinstance(vel_x_linear, list):
	    vel_x_instant = vel_x_linear[-window_size:]
	else:
	    vel_x_instant = [vel_x_linear]

	lin_vel = sum(vel_x_instant) / min(len(vel_x_instant), window_size)

	# repeat for the angular velocity
	if isinstance(vel_z_angular, list):
	    vel_z_instant = vel_z_angular[-window_size:]
	else:
	    vel_z_instant = [vel_z_angular]

	ang_vel = sum(vel_z_instant) / min(len(vel_z_instant), window_size)

	
def msg_function(required):
	global dist
	global lin_vel
	global ang_vel
	
	# print the values
	print("\n\nDistance from the desired position: {:.2f} m".format(dist))
	print("Robot average linear velocity along x: {:.2f} m/s".format(lin_vel))
	print("Robot average angular velocity along z: {:.2f} rad/s".format(ang_vel))
	
	return Average_pos_velResponse(dist, lin_vel, ang_vel)
	

if __name__ == '__main__':
	try:
		# initialize the node
		rospy.init_node('custom_msg_subscriber')
		
		# initialize the service
		s = rospy.Service('average_pos_vel', Average_pos_vel, msg_function)
		
		# inizialize the subscriber to the /pos_and_vel topic
		subscriber = rospy.Subscriber("/pos_and_vel", pos_and_vel, msg_callback)

		# execute in a loop
		rospy.spin()
			
	except rospy.ROSInterruptException:
		print("\n Error: program died unexpectedly", file=sys.stderr)
