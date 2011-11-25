#!/usr/bin/env python
# Initial code created by Graylin Trevor Jay (tjay@cs.brown.edu) an published under Crative Commens Attribution license.
# addition for signal interrupt by Koen Buys

import roslib; roslib.load_manifest('youbot_oodl')
import rospy
import tf
import tf.transformations
#import actionlib

from geometry_msgs.msg import *

from geometry_msgs.msg import Twist, PoseStamped


import sys, select, termios, tty, signal

if __name__=="__main__":	


	goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped)
	rospy.init_node('youbot_cmd_goal_python')

	print "starting node"


	try:
		ps = PoseStamped()

		ps.header.seq = 0
		ps.header.frame_id = "odom"
		ps.header.stamp = rospy.Time.now()
		ps.pose.position = geometry_msgs.msg.Point(1,0,0)
		ps.pose.orientation = Quaternion(0,0,0,1);
		print "publishing node"
		
		goal_publisher.publish(ps)

		print "node published"
		rospy.spin()

		
	except Exception, e:
		print e

