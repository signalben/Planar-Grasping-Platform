#!/usr/bin/env python
from utils.move_group import move
from utils.move_gripper import move_gripper
from utils.cnn_link import depthSub 
from utils.visual_feedback import plot_pose, rgbSub
from utils.projection import add_stamp, point_stamped_pub, pose_stamped_pub, projector 
import rospy
from sensor_msgs.msg import Image
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
import tf
from tf import transformations as ts
import scipy
from scipy.spatial.transform import Rotation
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
from cv_bridge import CvBridge, CvBridgeError

#this script impliments the top-level grasping routine for autonomous grasping trials

#list of prerequisites before grasping can be attempted
"""
roscore
rosrun scara_driver driver <- Requires arm to be connected via serial
rosrun scara_driver user <- Enable communications with '&E', home arm with '#$H'
rosrun scara_grasping joint_trajectory_action.py
 
roslaunch scara_bringup bringup.launch <- Expects Realsense to be plugged in 
roslaunch scara_grasping cnn.launch
rosrun scara_grasping execute_grasps.py
"""
full_FOV = 0 			#use all of FOV or just central portion
show_grasps = 1 		#visualize grasps within RGB image
start = [-2.1, -3.75, 0.09, 0]	#gripper position for observing gripper pose
out_view = [-2.1, -1, 0.09, 0]	#gripper position out of camera view so that grasps are not generated using imaging of the gripper

rospy.init_node('scara_main_control', anonymous=False) #this node has top-level control of the SCARA arm
rate = rospy.Rate(10)

#publishers for visualization of grasping
gripper_pose_pub = pose_stamped_pub("gripper_pose", "world", Point(0, 0, 0), Quaternion(0, 0, 0, 1))
grasp_pose_pub = pose_stamped_pub("grasp_pose", "world", Point(0, 0, 0), Quaternion(0, 0, 0, 1))
high_point_pub = point_stamped_pub("high_point", "world", Point(0, 0, 0))

#initialize helper classes for: getting RGB images, getting depth images, various tf functions, moving the arm, moving the gripper
rgbcam = rgbSub()
dcam = depthSub(full_FOV)
projector = projector()
scara = move()
gripper = move_gripper()

#initial offset used by the driver node to correct the uncalibrated jointstate AL  
rospy.set_param('gripper_yaw_offset', 0)

#the vertical distance to use between the grippers frame AL, and the grasp pose
gripper_jaw_offset = 0.150 # metres , smaller = lower

print("Initialised")

rospy.sleep(0.5)
scara.jnt_mov(out_view[0], out_view[1], out_view[2], out_view[3]) #Move out the way of the camera

#function obtains a single grasp by using the depth camera
def get_grasp():
	while not rospy.is_shutdown():		
		depth_img = dcam.get_img()				#Get new depth image
		grasp_pose, high_point = dcam.get_grasp(depth_img) 	#Blocks until a grasp is obtained from the depth image
		print("Getting grasps")
		if(grasp_pose is not None):
			print("Got grasp")
			jaw_opening_width = grasp_pose[4]				#Extract jaw opening width from stored grasp
			grasp_point = projector.point_from_image(grasp_pose, depth_img)	#Get grasp position within camera frame
			grasp_point = projector.map_points_to_world(grasp_point)	#Get grasp position within world frame
			high_point = projector.point_from_image(high_point, depth_img)	#Grasp Z height does not use local high point by default,
			high_point = projector.map_points_to_world(high_point)		#nevertheless it is published for visualization
			high_point_pub.update(high_point)
			high_point_pub.pub()
			#grasp_point.point.z = high_point.point.z			
			quat = projector.eul_to_quat([0, 0, grasp_pose[3]])		#get orientation of grasp as quaternion (only planar grasp - roll & pitch is always 0)
			quat = Quaternion(quat[0], quat[1], quat[2], quat[3]) 
			grasp_pose_pub.update_quat(quat)				#Publish grasp pose for visualization in RVIZ
			grasp_pose_pub.update_point(grasp_point)
			grasp_pose_pub.pub()
			rate.sleep()
			break

	return grasp_pose, jaw_opening_width, grasp_point	#returns planar grasp pose in image space, jaw width, grasp point in world frame

bridge = CvBridge()

#function opens window visualing grasp within an RGB image
def planar_show(grasp_pose, jaw_opening_width):
	image = rgbcam.get_img() #it gets a new RGB image for this purpose
	plot_pose(image, grasp_pose, jaw_opening_width) #overlay grasp on RGB images
	
	#black lines are drawn to indicate the FOV used by the CNN
	if(full_FOV == True):
		cv2.line(image, (80,   0), (80,  480), (0, 0, 0), 2)
		cv2.line(image, (560,  0), (560, 480), (0, 0, 0), 2)
	else:
		cv2.line(image, (170,  0), (170, 480), (0, 0, 0), 2)
		cv2.line(image, (470,  0), (470, 480), (0, 0, 0), 2)
		cv2.line(image, (0,   90), (640,  90), (0, 0, 0), 2)
		cv2.line(image, (0,  390), (640, 390), (0, 0, 0), 2)

	image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
	cv2.imshow('Frame',image)
	result = cv2.waitKey(10)

#function obtains the grippers position using RGB and depth images
def get_gripper_visual():
	while not rospy.is_shutdown():
		image = rgbcam.get_img()		 #Firstly get an RGB image
		gripper_pose = rgbcam.get_gripper(image) #then get the planar pose of gripper within that image (x,y,z,yaw), z is left empty
		depth_img = dcam.get_img()		 #then get a depth image
		if(gripper_pose is not None):
			gripper_point = projector.point_from_image(gripper_pose, depth_img) #As with grasps, get position in camera frame
			gripper_point = projector.map_points_to_world(gripper_point)	    #then convert to world frame
	
			quat = projector.eul_to_quat([0, 0, gripper_pose[3]])		    #Construct and send a 3D gripper pose for visualization 
			quat = Quaternion(quat[0], quat[1], quat[2], quat[3]) 
			gripper_pose_pub.update_quat(quat)
			gripper_pose_pub.update_point(gripper_point)
			gripper_pose_pub.pub()
			rate.sleep()
			break

	return gripper_pose, gripper_point	#returns the grippers planar pose in image space, gripper point in world frame  

#Gives a correcting offset to the driver node for gripper yaw (AL link), requires visually measured yaw value as argument  
def calibrate_gripper_rotation(observed_yaw):
	model_yaw = projector.get_estimated_gripper_yaw() #get the gripper yaw from tf, estimated by robot state publisher
	correction = observed_yaw - model_yaw		  #compute difference

	gripper_yaw_offset = 0
	if rospy.has_param('gripper_yaw_offset'):	  #if offset already exists, then update it		
		gripper_yaw_offset = rospy.get_param('gripper_yaw_offset')

	gripper_yaw_offset += correction		  #apply the correction
	rospy.set_param('gripper_yaw_offset', gripper_yaw_offset) #set the corrected offset


#allows a user to abort a grasp attempt before executing next motion of arm
def cancel():
	if(raw_input() != 'n'):
		return False

	print("Attempt aborted, restarting...")
	return True

#gives the user the option to ajust the grippers start position, to ensure it will be in view for visual localization step before attempting a grasp
while(1):
	userin = raw_input("Adjust gripper localization position? y/n")
	if(userin == 'y'):
		userin = raw_input("Enter adjustment for XJ in deg:")
		start[0] += float(int(userin))*3.14/180
		userin = raw_input("Enter adjustment for YJ in deg:")
		start[1] += float(int(userin))*3.14/180
		userin = raw_input("Enter adjustment for ZJ in mm:")
		start[2] += float(int(userin))/1000
		scara.jnt_mov(start[0], start[1], start[2], start[3])
		rospy.sleep(0.5)
	else:
		break

#main loop of grasping routine		
while not rospy.is_shutdown():
	print("Continue program? y/n")	#start of new grasp attempt
	if(cancel() == True):
		break
	while not rospy.is_shutdown():
		print("Starting new grasp attempt- Place object on the surface")
		scara.jnt_mov(out_view[0], out_view[1], out_view[2], out_view[3]) #Move gripper out the way of the camera
		rospy.sleep(0.5)

		print("Grasp this object? y/n")	#Confirm object to grasp is on work surface
		if(cancel() == True):
			break

		grasp_pose, jaw_opening_width, grasp_point = get_grasp() #Use depth camera to compute a grasp for the object
		if(show_grasps == True):
			planar_show(grasp_pose, jaw_opening_width)	#Optionally show grasp in RGB image
		
		scara.jnt_mov(start[0], start[1], start[2], start[3])	#Move gripper into camera view
		rospy.sleep(0.5)
		gripper_pose, gripper_point = get_gripper_visual()	#Observe the grippers position
		calibrate_gripper_rotation(gripper_pose[3])		#Correct the models gripper yaw
		rospy.sleep(0.5)
		
		#X,Y move to grasp point
		move_x = grasp_point.point.x - gripper_point.point.x
		move_y = grasp_point.point.y - gripper_point.point.y
		scara.cart_plan(move_x, move_y, 0)
		print("X,Y move to grasp point? y/n")
		print(move_x, move_y)
		if(cancel() == True):
			break
		scara.do_cart_plan()
		rospy.sleep(0.5)

		#W move to grasp angle
		move_yaw = grasp_pose[3] - projector.get_estimated_gripper_yaw()

		if(move_yaw > 0.5*math.pi):		#Align symmetric gripper at either 0 or 180 degrees, whichever is smaller
			move_yaw = move_yaw - math.pi

		elif(move_yaw < -0.5*math.pi):
			move_yaw = move_yaw + math.pi

		print("A move to grasp angle? y/n")
		print(move_yaw)
		if(cancel() == True):
			break
		scara.relative_jnt_mov(0, 0, 0, move_yaw)
		rospy.sleep(0.5)

		#Open gripper
		print("Open gripper? y/n")
		print(jaw_opening_width)
		if(cancel() == True):
			break
		gripper.set(jaw_opening_width)
		rospy.sleep(0.5)

		#Z move to grasp height
		move_z = grasp_point.point.z - gripper_point.point.z + gripper_jaw_offset
		plan = scara.cart_plan(0, 0, move_z)
		print("Z move to grasp height y/n")
		print(move_z)
		if(cancel() == True):
			break
		scara.do_cart_plan()
		rospy.sleep(0.5)

		#Close gripper 
		pwmval = 70	#PWM value sufficient for holding force between jaws
		print("Close gripper? y/n")
		print(pwmval)
		if(cancel() == True):
			break
		gripper.clasp(pwmval)
		rospy.sleep(0.5)

		#Raise Z
		plan = scara.cart_plan(0, 0, -move_z)
		print("Return? y/n")
		print(-move_z)
		if(cancel() == True):
			break
		scara.do_cart_plan()
		scara.jnt_mov(out_view[0], out_view[1], out_view[2], out_view[3]) #Move gripper out the way of the camera

		#Open gripper
		print("Drop? y/n")
		print(jaw_opening_width)
		if(cancel() == True):
			break
		gripper.set(jaw_opening_width)
		rospy.sleep(0.5)

		print("Attempt complete")




