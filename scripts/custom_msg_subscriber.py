#!/usr/bin/env python

import rospy
import math
import time

from RT1_second_assignment.msg import pos_and_vel

freq = rospy.get_param("publish_frequency")

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
	
	# save the linear and angular velocities of the robot
	lin_vel = msg.vel_x
	ang_vel = msg.vel_y
	
def print_message():
	global dist
	global lin_vel
	global ang_vel
	
	# print the values
	print("\n\nDistance from the desired position: {:.2f} m".format(dist))
	print("Robot linear velocity along x: {:.2f} m/s".format(lin_vel))
	print("Robot angular velocity along z: {:.2f} m/s".format(ang_vel))
	

if __name__ == '__main__':
	try:
		# initialize the node
		rospy.init_node('custom_msg_subscriber')
		
		# inizialize the subscriber to the /pos_and_vel topic
		subscriber = rospy.Subscriber("/pos_and_vel", pos_and_vel, msg_callback)
		
		# set the frequency as declared in the launch file (as parameter)
		rate = rospy.Rate(freq)
		
		while not rospy.is_shutdown():
			print_message()
			rate.sleep()
			
	except rospy.ROSInterruptException:
		print("\n Error: program died unexpectedly", file=sys.stderr)
