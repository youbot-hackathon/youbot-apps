#!/usr/bin/env python

import roslib; roslib.load_manifest('youbot_examples')
import rospy
import brics_actuator.msg

from brics_actuator.msg import JointPositions, JointValue, Poison

import sys, select, termios, tty, signal

if __name__=="__main__":
    
    pub = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions)
    rospy.init_node('simple_gripper_joint_position')
    rospy.sleep(0.5)
    
    try:
        jp = JointPositions()
        
        # if value == 0: gripper is closed
        # if value == 0.0115: griper is open
        
        jv1 = JointValue()
        jv1.joint_uri = "gripper_finger_joint_l"
        jv1.value = 0.01
        jv1.unit = "m"
        
        jv2 = JointValue()
        jv2.joint_uri = "gripper_finger_joint_r"
        jv2.value = 0.01
        jv2.unit = "m"
        
        
        p = Poison()
        jp.poisonStamp = p
        
        jp.positions = [jv1, jv2] #list
        
        pub.publish(jp)

        rospy.sleep(1.0)
    except Exception, e:
    	print e

