#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_examples')

import rospy
import threading
import tf
import time
import math
import geometry_msgs.msg
import kinematics_msgs.srv
import kinematics_msgs.msg
import sensor_msgs.msg
import motion_planning_msgs.msg
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

import sys

import Tkinter as tk
import termios, sys

from brics_actuator.msg import JointPositions, JointValue, Poison
import trajectory_msgs

class KinematicsTest:

    def __init__(self):
        self.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        self.configuration = [0, 0, 0, 0, 0]
        self.received_state = False

        rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.joint_states_callback)

        rospy.loginfo("Waiting for 'get_constraint_aware_ik' service")
        rospy.wait_for_service('/youbot_arm_kinematics/get_constraint_aware_ik')
        self.ciks = rospy.ServiceProxy('/youbot_arm_kinematics/get_constraint_aware_ik', kinematics_msgs.srv.GetConstraintAwarePositionIK)
        rospy.loginfo("Service 'get_constraint_aware_ik' is ready")

       
    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        for k in range(5):
            for i in range(len(msg.name)):
                joint_name = "arm_joint_" + str(k + 1)
                if(msg.name[i] == joint_name):
                    self.configuration[k] = msg.position[i]
        self.received_state = True


    def call_constraint_aware_ik_solver(self, goal_pose):
        while (not self.received_state):
            time.sleep(0.1)
        req = kinematics_msgs.srv.GetConstraintAwarePositionIKRequest()
        req.timeout = rospy.Duration(0.5)
        req.ik_request.ik_link_name = "arm_link_5"
        req.ik_request.ik_seed_state.joint_state.name = self.joint_names
        req.ik_request.ik_seed_state.joint_state.position = self.configuration
        req.ik_request.pose_stamped = goal_pose
        try:
            resp = self.ciks(req)
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: %s", str(e))
        return (resp.solution.joint_state.position, resp.error_code.val == motion_planning_msgs.msg.ArmNavigationErrorCodes.SUCCESS)

    def sendPosition(self, pose):
        # call IK solver
        (conf, success) = iks.call_constraint_aware_ik_solver(pose)
        if (success):
   
            print(conf)
            
            goal = control_msgs.msg.FollowJointTrajectoryGoal()
            
            goal.trajectory.joint_names = iks.joint_names
            
            jtp = trajectory_msgs.msg.JointTrajectoryPoint()
                
            for i in range(5):
                jtp.positions.append(conf[i])
            
            goal.trajectory.points.append(jtp)
                
            print "publishing cmd"
            armclient.send_goal(goal)
            
            armclient.wait_for_result()
            
            return True
        
        else:
            print("IK solver didn't find a solution")
            return False


    def sendGripper(self, left_val, right_val):
        # call IK solver
            
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        
        #goal.trajectory.joint_names.
        
        jtp = trajectory_msgs.msg.JointTrajectoryPoint()
           
        jtp.positions.append(left_val)
        jtp.positions.append(right_val)
        
        goal.trajectory.points.append(jtp)
            
        print "publishing gripper"
        gripperclient.send_goal(goal)
        
        gripperclient.wait_for_result()
        
        return True
    
def createPose(x, y, z, roll, pitch, yaw):
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pose.header.frame_id = "/arm_link_0"
        pose.header.stamp = rospy.Time.now()
        return pose
       

def getChar():
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
         
if __name__ == "__main__":
    rospy.init_node('cartesian_arm_command')
    time.sleep(0.5)
    
    armclient = actionlib.SimpleActionClient("/arm_1/arm_controller/joint_trajectory_action", 
                                             control_msgs.msg.FollowJointTrajectoryAction)
    print "waiting for arm action server"
    armclient.wait_for_server()
    
    gripperclient = actionlib.SimpleActionClient("/arm_1/gripper_controller/joint_trajectory_action", 
                                             control_msgs.msg.FollowJointTrajectoryAction)
    print "waiting for gripper action server"
    gripperclient.wait_for_server()
    
    iks = KinematicsTest()
    

    # Grasp from floor in front
    x =  0.05
    y =  0.0
    z =  0.3
    #z = 0.05
    roll = 0
    pitch = 0#-math.pi/2
    yaw = 0
 
    x = 0.024 + 0.033
    y = 0
    z = 0.535
    roll = 0
    pitch = 0
    yaw = 0
    
    target = createPose(x, y, z, roll, pitch, yaw)
    origin = target
    
    deltaC = 0.002
    deltaR = 0.02
    deltaG = 0.002
    
    gripper_left = 0.00
    gripper_right = 0.00
    
    iks.sendPosition(origin)
    
    print "reading commands"
    
    while (True):
        oldtarget = target
        
        c = getChar()
        
        if (c == "w"):
            x += deltaC
        if (c == "s"):
            x -= deltaC
        
        if (c == "a"):
            y += deltaC
        if (c == "d"):
            y -= deltaC
        
        if (c == "e"):
            z += deltaC
        if (c == "q"):
            z -= deltaC
        
        if (c == "i"):
            roll += deltaR
        if (c == "k"):
            roll -= deltaR
        
        if (c == "j"):
            pitch += deltaR
        if (c == "l"):
            pitch -= deltaR
          
        if (c == "u"):
            yaw += deltaR
        if (c == "o"):
            yaw -= deltaR
        
        if (c == "g"):
            gripper_left += deltaR
            gripper_right += deltaR
            
        if (c == "v"):
            gripper_left -= deltaG
            gripper_right -= deltaG
        
        if (c == "r"):
            x = origin.pose.position.x
            y = origin.pose.position.y
            z = origin.pose.position.y
            roll = 0
            pitch = 0
            yaw = 0
            
        if (c == "t"):
            break
        if (c == "<Esc>"):
            break
        
        target = createPose(x, y, z, roll, pitch, yaw)
    
        print target
        print "Gripper: ", gripper_left, " - ", gripper_right
            
        pose = createPose(x, y, z, roll, pitch, yaw)
        
        succ = iks.sendPosition(pose)
        
        iks.sendGripper(gripper_left, gripper_left)
        
            
    
    #while(True):
        
    #    c = sys.stdin.read(1)
        
    #    print "Got key", c
        
        #pose = iks.createPose(x, y, z, roll, pitch, yaw)
        
        #iks.sendPosition(pose)
    