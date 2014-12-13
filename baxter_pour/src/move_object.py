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


class BaxterMove(object):

    _feedback = baxter_pour.msg.MoveFeedback()
    _result   = baxter_pour.msg.MoveResult()

    def __init__(self,name):
        print 'initializing'

        self.approach_distance = 0 #.07
        self.elevated_distance = .3
        
        #Find the baxter from the parameter server
        self.baxter = URDF.from_parameter_server() 
        #Note:  A node that initializes and runs the baxter has to be running in the background for 
        #       from_parameter_server to to find the parameter.
        #       Older versions of URDF label the function "load_from_parameter_server"


        #Subscribe to the "baxter1_joint_states" topic, which provides the joint states measured by the baxter in a
        # ROS message of the type sensor_msgs/JointState. Every time a message is published to that topic run the 
        #callback function self.get_joint_states, which is defined below.
        #self.joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.get_joint_states)
        self.end_eff_state_sub = rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,self.get_end_eff_state)
        
        self.listener=tf.TransformListener()

        #initialize gripper
        self.initialize_gripper()
        # self.timer1 = rospy.Timer(rospy.Duration(0.01),) 

        #calibrate the gripper
        self.left_gripper.calibrate()

        #Get pick up and drop off coords
        #I.e subscr. or ...


        self._action_name = name
        #The main function of BaxterMove is called whenever a client sends a goal to move_server
        self._as = actionlib.SimpleActionServer(self._action_name, baxter_pour.msg.MoveAction, execute_cb=self.main, auto_start = False)
        self._as.start()

        #To test the node separatly call self.main() manually
        #self.main()

        return 

    def main(self,goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        #define your limb
        self.arm = 'left'

        #get pick-up and drop-off data
        self.pickup_data = numpy.array([goal.x_pickup, goal.y_pickup, goal.z_pickup,1])
        print 'received pick-up data: ', self.pickup_data
        self.dropoff_data=numpy.array([goal.x_dropoff, goal.y_dropoff, goal.z_dropoff,1])


        # move to offset
        # self.move_to_approach_pose1(self.pickup_data)
        self.move_to_approach_pose(self.pickup_data)

        #approach, grasp, lift
        self.pickup_object()

        #move away from table to an elevated pick up waypoint
        current_pose=self.end_eff_state.pose.position
        current_pose = numpy.array([current_pose.x,current_pose.y,current_pose.z,1])
        self.move_to_approach_pose(current_pose)

        #move to 'approach position' of elevated drop off waypoint
        elevated_dropoff_waypoint = self.dropoff_data
        # elevated_dropoff_waypoint[2]=elevated_dropoff_waypoint[2]+self.elevated_distance
        self.move_to_approach_pose(elevated_dropoff_waypoint)

        # approach,lower,release
        self.drop_off_object()
        # move to approach position 

        self._result.status=True

        if success:
          rospy.loginfo('%s: Succeeded' % self._action_name)
          self._as.set_succeeded(self._result)     


    def move_to_approach_pose(self,pose):

        approach_pose=pose
        approach_pose[0]=approach_pose[0]-self.approach_distance
        approach_pose[2]=approach_pose[2]+self.elevated_distance


        approach_configuration=self.ik(self.arm,approach_pose)
        self.cmd_joint_angles(approach_configuration)


    def pickup_object(self):

        #move forward to grasp position
        current_pose=self.end_eff_state.pose.position
        grasp_pose = numpy.array([current_pose.x,current_pose.y,current_pose.z,1])
        grasp_pose[0]=grasp_pose[0]+self.approach_distance

        grasp_pose[2]=grasp_pose[2]-self.elevated_distance

        grasp_configuration=self.ik(self.arm,grasp_pose)
        self.cmd_joint_angles(grasp_configuration) 
        time.sleep(2)
        #grasp object
        self.close_gripper()
        time.sleep(2)

        #lift oject to elevated position
        # current_pose=self.end_eff_state.pose.position
        # elevated_pose = numpy.array([current_pose.x,current_pose.y,current_pose.z,1])
        # elevated_pose[2]=elevated_pose[2]+self.elevated_distance
        # elevated_configuration=self.ik(self.arm,elevated_pose)
        # self.cmd_joint_angles(elevated_configuration)
        time.sleep(2)

    def drop_off_object(self):
        #move forward to elevated drop-off waypoint
        # current_pose=self.end_eff_state.pose.position
        # elevated_pose = numpy.array([current_pose.x,current_pose.y,current_pose.z,1])
        # elevated_pose[0] = elevated_pose[0]+self.approach_distance
        # elevated_configuration=self.ik(self.arm,elevated_pose)
        # self.cmd_joint_angles(elevated_configuration)
        # time.sleep(2)

        #lower arm to drop-off waypoint
        current_pose=self.end_eff_state.pose.position
        release_pose = numpy.array([current_pose.x,current_pose.y,current_pose.z,1])
        release_pose[2] =release_pose[2]-self.elevated_distance
        release_pose[0] =release_pose[0]+self.approach_distance
        release_configuration=self.ik(self.arm,release_pose)
        self.cmd_joint_angles(release_configuration)
        time.sleep(2)

        #open gripper
        self.open_gripper()
        time.sleep(2)

        #move back to drop-off 'aproach' waypoint
        current_pose=self.end_eff_state.pose.position
        drop_off_pose=numpy.array([current_pose.x,current_pose.y,current_pose.z,1])
        self.move_to_approach_pose(drop_off_pose)


    def initialize_gripper(self):
        # initialize interfaces
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)



    def ik(self,limb, cmd_pos):
        self.arg_fmt = argparse.RawDescriptionHelpFormatter
        self.parser = argparse.ArgumentParser(formatter_class=self.arg_fmt,
                                         description=main.__doc__)
        
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

    def cmd_joint_angles(self, data):
        self.limb = baxter_interface.Limb(self.arm)
        self.limb.move_to_joint_positions(data)
        print "i'm trying to move"
        pass

    def get_end_eff_state(self,data):
        
        try:
            self.end_eff_state=data
            # print self.end_eff_state
        except  rospy.ROSInterruptException:
            self.end_eff_state = None
            pass

        return   

    def close_gripper(self):
        self.left_gripper.close()
        pass

    def open_gripper(self):
        self.left_gripper.open()
        pass 

def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('move_server')

    try:
        demo = BaxterMove(rospy.get_name())
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__=='__main__':
    main()