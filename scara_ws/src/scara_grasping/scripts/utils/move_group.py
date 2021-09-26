#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf import TransformListener
from tf import transformations as ts


#this class allows control of the manipulator in Cartestian and joint modes
class move:

	#uses the MoveIt commander Python wrappers for MoveGroup
	#'arm' is the only group controlled, and does not include the gripper
    def __init__(self):
	moveit_commander.roscpp_initialize(sys.argv)
	self.robot = moveit_commander.RobotCommander()
	self.arm = moveit_commander.MoveGroupCommander("arm")
	self.scene = moveit_commander.PlanningSceneInterface()
	self.listener = TransformListener()
	group_names = self.robot.get_group_names()
	print("Available Planning Groups:", group_names)

	#allows specification of relative Cartesian motion of the end-effector in metres
    def cart_plan(self, x, y, z):
	rospy.sleep(1)
	waypoints = []
	wpose = self.arm.get_current_pose().pose #get x,y,z,q1,q2,q3,q4 pose of end-effector

	(trans,rot) = self.listener.lookupTransform('world', 'AL', rospy.Time(0)) #update positions, as sometimes get_current_pose() lags
	wpose.position.x = trans[0]
	wpose.position.y = trans[1]
	wpose.position.z = trans[2]

	waypoints.append(copy.deepcopy(wpose)) #nessessary to add start pose to list 
	
	wpose.position.x = wpose.position.x + x 
	wpose.position.y = wpose.position.y + y
	wpose.position.z = wpose.position.z + z

	if wpose.position.z < 0.157: #if below table, don't be below table
		print("Z too low - will hit the table :" + str(wpose.position.z))
		wpose.position.z = 0.157

	waypoints.append(copy.deepcopy(wpose)) #desired pose last item in list
	(self.plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0) #produce plan (can be viewed in Rviz), but don't execute yet

    def do_cart_plan(self): #execute previously generated Cartesian plan
	self.arm.execute(self.plan, wait=True)        
	self.arm.stop()
     
    def jnt_mov(self, x, y, z, a): #set joints in Radians, absolute position
	joint_goal = self.arm.get_current_joint_values()
	joint_goal[0] = z
	joint_goal[1] = x
	joint_goal[2] = y
	joint_goal[3] = a
	self.arm.go(joint_goal, wait=True) #executes imeadietly
	self.arm.stop()

    def relative_jnt_mov(self, x, y, z, a): #set joints in Radians, relative move
	joint_goal = self.arm.get_current_joint_values()
	joint_goal[0] += z
	joint_goal[1] += x
	joint_goal[2] += y
	joint_goal[3] += a
	self.arm.go(joint_goal, wait=True) #executes imeadietly
	self.arm.stop()





"""
-References:

Adapted from guide explaning moveit commander:
http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/moveit_commander_scripting/moveit_commander_scripting_tutorial.html

Method of coping with get_current_pose() lag
https://answers.ros.org/question/300978/movegroupcommander-get_current_pose-returns-incorrect-result-when-using-real-robot/
https://groups.google.com/g/moveit-users/c/kZaVTrlsUc0

"""

	



	

