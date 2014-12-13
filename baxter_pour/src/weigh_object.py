#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion, compose_matrix
from geometry_msgs.msg import Twist, Pose, PoseStamped,Point,Quaternion,TransformStamped
from sensor_msgs.msg import JointState,Range
from urdf_parser_py.urdf import URDF
from baxter_core_msgs.msg import EndpointState 
from baxter_interface import CHECK_VERSION
import baxter_interface
import time
import numpy
#From baxter_example ik service
import argparse 
import struct
import sys
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import roslib
import actionlib
import baxter_pour.msg

class BaxterWeigh(object):

    _feedback = baxter_pour.msg.WeighFeedback()
    _result   = baxter_pour.msg.WeighResult()

    def __init__(self,name):

        self.elevated_distance = .3 #.07

        #Find the baxter from the parameter server
        self.baxter = URDF.from_parameter_server() 
        #Note:  A node that initializes and runs the baxter has to be running in the background for 
        #       from_parameter_server to to find the parameter.
        #       Older versions of URDF label the function "load_from_parameter_server"


        #Subscribe to the "baxter1_joint_states" topic, which provides the joint states measured by the baxter in a
        # ROS message of the type sensor_msgs/JointState. Every time a message is published to that topic run the 
        #callback function self.get_joint_states, which is defined below.
        self.joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.get_joint_states)
        self.end_eff_state_sub = rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,self.get_end_eff_state)
        
        self.listener=tf.TransformListener()
        # self.timer1 = rospy.Timer(rospy.Duration(0.01),) 

        #calibrate the gripper
        self.initialize_gripper()


        self._action_name = name
        #The main function of BaxterWeigh is called whenever a client sends a goal to weigh_server
        self._as = actionlib.SimpleActionServer(self._action_name, baxter_pour.msg.WeighAction, execute_cb=self.main, auto_start = False)
        self._as.start()

        #To test the node separatly call self.main() manually
        # self.main()
        return  

    def main(self,goal):
                # helper variables
        r = rospy.Rate(1)
        success = True  

        #define your limb
        self.arm = 'left'

        #Get pickup location from client
        pickup_data = numpy.array([goal.x_pickup, goal.y_pickup, goal.z_pickup,1])

        # move to offset
        self.move_to_approach_pose(pickup_data)

        #approach, grasp, lift, weigh, put back, go to approach pose
        weight=self.weigh_object()
        self._result.weight=weight  

        if success:
          rospy.loginfo('%s: Reported object weight of...' % self._action_name)
          self._as.set_succeeded(self._result)

    def move_to_approach_pose(self,pose):

        approach_pose=pose
        approach_pose[2]=approach_pose[2]+self.elevated_distance
        approach_configuration=self.ik(self.arm,approach_pose)
        self.cmd_joint_angles(approach_configuration)


    def initialize_gripper(self):
        # initialize interfaces
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)

        self.left_gripper.calibrate()

    def weigh_object(self):

        #move to grasp position
        current_pose=self.end_eff_state.pose.position
        grasp_pose = numpy.array([current_pose.x,current_pose.y,current_pose.z,1])
        grasp_pose[2]=grasp_pose[2]-self.elevated_distance
        grasp_configuration=self.ik(self.arm,grasp_pose)
        self.cmd_joint_angles(grasp_configuration) 
        time.sleep(2)

        #get initial force in z
        time.sleep(2)
        initial_weight=self.end_eff_state.wrench.force.z

        #close the gripper
        self.close_gripper()
        #raises the object a small amount to evaluate weight at the end effector
        self.delta_z_desired=0.05

        #get the current position
        current_pose=self.end_eff_state.pose.position

        #set the new position with a small delta
        elevated_pose = numpy.array([current_pose.x,current_pose.y,current_pose.z,1])
        elevated_pose[2] = elevated_pose[2] + self.delta_z_desired

        #run the IK and move
        new_joint_angles = self.ik(self.arm,elevated_pose)
        self.cmd_joint_angles(new_joint_angles)

        time.sleep(2)
        #get new force in z and print weight
        new_weight=self.end_eff_state.wrench.force.z

        time.sleep(1)

        actual_weight=new_weight-initial_weight
        print 'Reported weight:',actual_weight

        #get the current position
        lowered_pose=elevated_pose
        lowered_pose[2] = lowered_pose[2] - self.delta_z_desired

        #run the IK and move
        new_joint_angles = self.ik(self.arm,lowered_pose)
        self.cmd_joint_angles(new_joint_angles)
        time.sleep(2)


        #open gripper 
        self.open_gripper()
        time.sleep(1)       

        #move back to approach position 
        current_pose=self.end_eff_state.pose.position
        current_pose = numpy.array([current_pose.x,current_pose.y,current_pose.z,1])
        self.move_to_approach_pose(current_pose)

        return new_weight

    def cmd_joint_angles(self, data):
        self.limb = baxter_interface.Limb(self.arm)
        self.limb.move_to_joint_positions(data)
        print "i'm trying to move"
        pass



    def ik(self,limb, cmd_pos):
        self.arg_fmt = argparse.RawDescriptionHelpFormatter
        self.parser = argparse.ArgumentParser(formatter_class=self.arg_fmt,
                                         description=main.__doc__)

        # t = self.listener.getLatestCommonTime('base', 'kinect') Better solution. Returns 0. Lear proper way: posestamped, frame_ID etc 
        # try:
        #     time.sleep(1)
        #     (self.transl,self.quat)=self.listener.lookupTransform('base','kinect',rospy.Time(0))
        #     self.rot = euler_from_quaternion(self.quat)
        #     self.tf_SE3 = compose_matrix(angles=self.rot,translate = self.transl)
        #     self.cmd_pos=numpy.dot(self.tf_SE3,kinect)
        #     print self.tf_SE3
        #     # print self.cmd_pos
        # except (tf.Exception):
        #     rospy.logerr("Could not transform from "\
        #                  "{0:s} to {1:s}".format(base,kinect))
        #     return
        
        self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
        self.ikreq = SolvePositionIKRequest()
        self.hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        
        self.poses = {
            'left': PoseStamped(
                header=self.hdr,
                pose=Pose(
                    position=Point(
                        x=cmd_pos[0],
                        y=cmd_pos[1],
                        z=cmd_pos[2]
                    ),
                    orientation=Quaternion(

                        x=1,
                        y=0,
                        z=0,
                        w=0,                     
                    ),
                ),
            ),
            'right': PoseStamped(
                header=self.hdr,
                pose=Pose(
                    position=Point(
                        x=0.656982770038,
                        y=-0.852598021641,
                        z=0.0388609422173,
                    ),
                    orientation=Quaternion(
                        x=0.367048116303,
                        y=0.885911751787,
                        z=-0.108908281936,
                        w=0.261868353356,
                    ),
                ),
            ),
        }

        self.ikreq.pose_stamp.append(self.poses[limb])
        try:
            rospy.wait_for_service(self.ns, 5.0)
            self.resp = self.iksvc(self.ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        self.resp_seeds = struct.unpack('<%dB' % len(self.resp.result_type),
                                   self.resp.result_type)

        if (self.resp_seeds[0] != self.resp.RESULT_INVALID):
            self.seed_str = {
                        self.ikreq.SEED_USER: 'User Provided Seed',
                        self.ikreq.SEED_CURRENT: 'Current Joint Angles',
                        self.ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(self.resp_seeds[0], 'None')
            # print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
            #       (self.seed_str,))
            # Format solution into Limb API-compatible dictionary
            print 'limb_joints'
            self.limb_joints = dict(zip(self.resp.joints[0].name, self.resp.joints[0].position))
            #print "\nIK Joint Solution:\n", self.limb_joints
            #print "------------------"
            #print "Response Message:\n", self.resp

            # print self.limb_joints

        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
        return self.limb_joints


    def close_gripper(self):
        self.left_gripper.close()
        pass

    def open_gripper(self):
        self.left_gripper.open()
        pass

    def get_joint_states(self,data):
        try:
            self.q_sensors = data.position
        except rospy.ROSInterruptException: 
            self.q_sensors = None
            pass
        return

    def get_end_eff_state(self,data):
        
        try:
            self.end_eff_state=data
            # print self.end_eff_state
        except  rospy.ROSInterruptException:
            self.end_eff_state = None
            pass

        return


def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('weigh_server')

    try:
        demo = BaxterWeigh(rospy.get_name())
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__=='__main__':
    main()


