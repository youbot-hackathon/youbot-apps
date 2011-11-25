#! /usr/bin/env python

import roslib; roslib.load_manifest('youbot_examples')
import rospy
import actionlib

from move_base_msgs.msg import *
from geometry_msgs.msg import *

if __name__ == '__main__':
    try:
        rospy.init_node('simple_navigation_goals_python')

        # create action client and wait for the server
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
    
        g1 = MoveBaseGoal()
        g1.target_pose.header.frame_id = 'base_link'
        g1.target_pose.pose.position.x = 1.0
        g1.target_pose.pose.position.y = 1.0
        
        g1.target_pose.pose.orientation.w = 1.0
        
        client.send_goal(g1)
        client.wait_for_result()
    
        print "done..."

    except rospy.ROSInterruptException, e:
        print e
