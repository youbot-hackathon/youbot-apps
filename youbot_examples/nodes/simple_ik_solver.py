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

from brics_actuator.msg import JointPositions, JointValue, Poison

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



if __name__ == "__main__":
    rospy.init_node('youbot_ik_solver_test')
    time.sleep(0.5)
    
    armpub = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions)
    
    iks = KinematicsTest()
    

    # Grasp from floor in front
    x =  0.25
    y =  0.0
    z = 0.15
    roll = 0
    pitch = -math.pi / 2
    yaw = 0
 
    x = 0.024 + 0.033 + 0.4
    y = 0
    z = 0.115
    roll = 0
    pitch = math.pi / 2.0
    yaw = 0
    
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
    
    # (conf, success) = iks.call_ik_solver(pose)
    (conf, success) = iks.call_constraint_aware_ik_solver(pose)
    if (success):
        # publish solution directly as joint positions
        print(conf)
        jp = JointPositions()
        
        for i in range(5):
            jv = JointValue()
            jv.joint_uri = iks.joint_names[i]
            jv.value = conf[i]
            jv.unit = "rad"
            jp.positions.append(jv)
        
        rospy.sleep(0.5)
        print "publishing cmd"
        armpub.publish(jp)
        
        
        
    else:
        print("IK solver didn't find a solution")