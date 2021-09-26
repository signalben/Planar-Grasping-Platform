#!/usr/bin/env python
from utils.move_group import *
from utils.move_gripper import *
from utils.cnn_link import *
from utils.visual_feedback import *
from utils.projection import *
import rospy
from sensor_msgs.msg import Image
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
import tf
from tf import transformations as ts
import copy
import scipy
from scipy.spatial.transform import Rotation
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
from cv_bridge import CvBridge, CvBridgeError

#this script generates grasps by the same method as the grasping routine in execte_grasps.py, but is only for visualization/inpection purposes
#functions for control of the manipulator have been removed

full_FOV = 0 			#use all of FOV or just central portion
show_grasps = 1 		#visualize grasps within RGB image

rospy.init_node('view_grasps', anonymous=True)
rate = rospy.Rate(10)
gripper_pose_pub = pose_stamped_pub("gripper_pose", "world", Point(0, 0, 0), Quaternion(0, 0, 0, 1))
grasp_pose_pub = pose_stamped_pub("grasp_pose", "world", Point(0, 0, 0), Quaternion(0, 0, 0, 1))
high_point_pub = point_stamped_pub("high_point", "world", Point(0, 0, 0))

#initialize helper classes for: getting RGB images, getting depth images, various tf functions
rgbcam = rgbSub()
dcam = depthSub(full_FOV)
projector = projector()
rospy.set_param('gripper_yaw_offset', 0)
print("view_grasps Initialised")

#function obtains a single grasp by using the depth camera
def get_grasp():
	while not rospy.is_shutdown():
		depth_img = dcam.get_img()				#Get new depth image
		grasp_pose, high_point = dcam.get_grasp(depth_img)	#Blocks until a grasp is obtained from the depth image
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

#main loop continously gets grasps for visualization
while not rospy.is_shutdown():

	grasp_pose, jaw_opening_width, grasp_point = get_grasp()

	#open window visualing grasp within an RGB image
	if(show_grasps == True):
		image = rgbcam.get_img()  #it gets a new RGB image for this purpose
		plot_pose(image, grasp_pose, jaw_opening_width)

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
		
	

