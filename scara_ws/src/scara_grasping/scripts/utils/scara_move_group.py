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



class move:

    def __init__(self):
	moveit_commander.roscpp_initialize(sys.argv)
	self.robot = moveit_commander.RobotCommander()
	self.arm = moveit_commander.MoveGroupCommander("arm")
	self.scene = moveit_commander.PlanningSceneInterface()
	#force_update = self.arm.get_current_pose().pose
	self.listener = TransformListener()

	# We can get the name of the reference frame for this robot:
	planning_frame = self.arm.get_planning_frame()
	print("============ Planning frame: %s" % planning_frame)

	# We can also print the name of the end-effector link for this group:
	eef_link = self.arm.get_end_effector_link()
	print("============ End effector link: %s" % eef_link)

	# We can get a list of all the groups in the robot:
	group_names = self.robot.get_group_names()
	print("============ Available Planning Groups:", group_names)

	# Sometimes for debugging it is useful to print the entire state of the
	# robot:
	#print("============ Printing robot state")
	#print(self.robot.get_current_state())
	#print("")

	"""
    def cart_plan(self, x, y, z):
	pose = copy.deepcopy(self.arm.get_current_pose().pose)
	pose.position.x -= x  
	pose.position.y += y   
	pose.position.z += z 

	self.arm.set_pose_target(pose)
     
    def do_cart_plan(self): 
	result = self.arm.go(wait=True)     
	self.arm.stop()
	self.arm.clear_pose_targets()  

    def cart_mov(self, x, y, z): 
	self.do_cart_plan(self.cart_plan(x, y, z)) 
	"""
    def cart_plan(self, x, y, z):
	rospy.sleep(1)
	waypoints = []
	wpose = self.arm.get_current_pose().pose

	#get_estimated_gripper_point
	#rospy.sleep(1)
	(trans,rot) = self.listener.lookupTransform('world', 'AL', rospy.Time(0))
	wpose.position.x = trans[0]
	wpose.position.y = trans[1]
	wpose.position.z = trans[2]


	waypoints.append(copy.deepcopy(wpose))
	
	print("==============================================ORIG")
	print(wpose.position.z)
	print("==============================================RECIEVED Z")
	print(z)
	wpose.position.x = wpose.position.x + x 
	wpose.position.y = wpose.position.y + y
	wpose.position.z = wpose.position.z + z
	print("==============================================NEW Z")
	print(wpose.position.z)

	if wpose.position.z < 0.1631:#6mm safety
		print("Z too low - will hit the table :" + str(wpose.position.z))
		wpose.position.z = 0.1631
	else:
		print("Z fine")
		print(wpose.position.z)
		print(wpose.position.z < 0.1631)
		print(wpose.position.z > 0.1631)  

	waypoints.append(copy.deepcopy(wpose))
	(self.plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0) 

    def do_cart_plan(self): 
	self.arm.execute(self.plan, wait=True)        
	self.arm.stop()
     

    def jnt_mov(self, x, y, z, a): 
	joint_goal = self.arm.get_current_joint_values()
	joint_goal[0] = z
	joint_goal[1] = x
	joint_goal[2] = y
	joint_goal[3] = a
	self.arm.go(joint_goal, wait=True)
	self.arm.stop()

    def relative_jnt_mov(self, x, y, z, a): 
	joint_goal = self.arm.get_current_joint_values()
	joint_goal[0] += z
	joint_goal[1] += x
	joint_goal[2] += y
	joint_goal[3] += a
	self.arm.go(joint_goal, wait=True)
	self.arm.stop()







	

