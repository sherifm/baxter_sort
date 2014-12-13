#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import compose_matrix
from tf.transformations import is_same_transform
from geometry_msgs.msg import Twist, Pose, PoseStamped,Point,Quaternion
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from baxter_pykdl.baxter_pykdl import baxter_kinematics
from baxter_core_msgs.msg import EndpointState 
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

import numpy

class BaxterPour:
	def __init__(self):
		print 'initializing'
		
		#Find the baxter from the parameter server
		self.baxter = URDF.from_parameter_server() 
		#Note: 	A node that initializes and runs the baxter has to be running in the background for 
		#		from_parameter_server to to find the parameter.

		
		#		Older versions of URDF label the function "load_from_parameter_server"


		#Subscribe to the "baxter1_joint_states" topic, which provides the joint states measured by the baxter in a
		# ROS message of the type sensor_msgs/JointState. Every time a message is published to that topic run the 
		#callback function self.get_joint_states, which is defined below.
		self.joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.get_joint_states)
		self.end_eff_state_sub = rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,self.get_end_eff_state)
		


		self.baxterkdl=baxter_kinematics('left')
		self.timer1 = rospy.Timer(rospy.Duration(0.01), self.inverse_kinematics) 

		

		return	

	def get_joint_states(self,data):
		try:
			self.q_sensors = data.position
		except rospy.ROSInterruptException: 
			self.q_sensors = None
			pass
		
		return

	def get_end_eff_state(self,data):
		
		try:
			self.end_eff_state=data.pose
			# print self.end_eff_state
		except  rospy.ROSInterruptException:
			self.end_eff_state = None
			pass

		return

	def inverse_kinematics(self,data):
            hdr = Header(stamp=rospy.Time.now(), frame_id='base')
            poses = {
                'left': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position=Point(
                            x=0.657579481614,
                            y=0.851981417433,
                            z=0.0388352386502,
                        ),
                        orientation=Quaternion(
                            x=-0.366894936773,
                            y=0.885980397775,
                            z=0.108155782462,
                            w=0.262162481772,
                        ),
                    ),
                ),
                'right': PoseStamped(
                    header=hdr,
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

	

	   # rospy.wait_for_service('/ExternalTools/left/PositionKinematicsNode/IKService')
	   # ik = rospy.ServiceProxy('/ExternalTools/left/PositionKinematicsNode/IKService',SolvePositionIK)

	    # result = ik(self.poses)
	    print self.poses

	    # print ik


def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('baxter_pour')

    try:
        demo = BaxterPour()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__=='__main__':
    main()
