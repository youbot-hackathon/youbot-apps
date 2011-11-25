#!/usr/bin/env python

import roslib; roslib.load_manifest('youbot_examples')
import rospy
import sys, select, termios, tty, signal

from geometry_msgs.msg import Twist

if __name__=="__main__":
    		
	pub = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('simple_cmd_cel_python')

	try:
		while not rospy.is_shutdown():
			# set velocities
			twist = Twist()
			twist.linear.x = 1
			twist.linear.y = 0
			twist.linear.z = 0

			twist.angular.x = 0 
			twist.angular.y = 0
			twist.angular.z = 0

			# publish the topic
			pub.publish(twist)

			# wait 0.1s
			rospy.sleep(0.1)
	except:
		print e

	finally:
		# stop the robot
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
