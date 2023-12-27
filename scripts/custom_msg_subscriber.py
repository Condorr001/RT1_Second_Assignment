#!/usr/bin/env python

import rospy
import math
import time

from RT1_second_assignment.msg import pos_and_vel

freq = rospy.get_param("publish_frequency")
dist = 0
vel = 0


def msg_callback(msg):
	global dist
	global vel
	# get the the desired position of the robot
	x_desired = rospy.get_param("des_pos_x")
	y_desired = rospy.get_param("des_pos_y")
	
	# get the current position of the robot
	x = msg.x
	y = msg.y
	
	# compute the distance between the robot and the desired position
	dist = math.dist([x_desired, y_desired], [x, y])
	
	# compute the average speed of the robot
	vel = math.sqrt(msg.vel_x**2 + msg.vel_y**2)
	
def print_message():
	global dist
	global vel
	# print the values
	print("\n\nDistance from the desired position: {:.2f} m".format(dist))
	print("Robot average speed: {:.2f} m/s".format(vel))
	

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
