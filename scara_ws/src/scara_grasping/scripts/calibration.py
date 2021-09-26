#!/usr/bin/env python

from utils.move_group import move
from utils.move_gripper import move_gripper
from utils.cnn_link import depthSub 
from utils.visual_feedback import plot_pose, rgbSub
from utils.projection import add_stamp, point_stamped_pub, pose_stamped_pub, projector 

import rospy
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import tf
from tf import transformations as ts
import scipy
from scipy.spatial.transform import Rotation
import numpy as np

#this script obtains the 6DOF pose the RGB-D camera by observation of the grippers movement
#requires that the static_tranform_publisher in bringup.launch is disabled 

full_FOV = 1
square_width = 0.08 #gripper moves in a sqaure, is the lenth of square in m

rospy.init_node('scara_main_control', anonymous=True) #this node has top-level control of the SCARA arm
rate = rospy.Rate(10)

#initialize helper classes for: moving the arm, getting RGB images, getting depth images, various tf functions
scara = move()
rgbcam = rgbSub()
dcam = depthSub(full_FOV)
projector = projector()
print("Initializing...")

#start with an approximate camera tranform
camera_trans = [0.2, 0.2, 0.5]
projector.set_camera_trans(camera_trans)

camera_rot =  [-1.57, 1.57, 0.4]
camera_quat = projector.eul_to_quat(camera_rot)
projector.set_camera_rot(camera_quat)	
rate.sleep()

#begins by moving gripper in sight of camera for identification		
raw_input("Press Enter to begin calibration...")
start = [-2.1, -3.85, 0.081, 0]
scara.jnt_mov(start[0], start[1], start[2], start[3])

#gives the user the option to ajust the grippers start position so that it remains in view for all motions
while(1):
	userin = raw_input("Adjust joints? y/n")
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
 
measured = [] #list of points from gripper observation
estimated = [] #lis of points from robot state publisher

#list of relative motions forming a sqaure
xy_calibration_motions = [[square_width, 0], [0, square_width], [-square_width, 0], [0, -square_width]]

#for each motion
for motion in xy_calibration_motions:
	scara.cart_plan(motion[0], motion[1], 0) 	#plan a cartestian move
	scara.do_cart_plan() 				#execute cartesian move
	rospy.sleep(0.5)				#be certain motion has finished before getting new position
	point = projector.get_estimated_gripper_point()	#get new position
	point.z += 0.02					#2cm Z offset between fiducial marker and gripper frame AL
	estimated.append(point)				#store robot state publishers estimated points

	while (True):		#blocks until a new RGB image, depth image, grippers position in the image are obtained
		rgb_image = rgbcam.get_img()	
		planar_grip_pose = rgbcam.get_gripper(rgb_image)
		depth_img = dcam.get_img()
		point = projector.point_from_image(planar_grip_pose, depth_img)	#convert gripper position from camera space to Cartesian
		if point:
			measured.append(point) 		#store these observed points
			break
	rate.sleep()

scara.jnt_mov(start[0], start[1], start[2], start[3])	#put gripper back at start position (should be here anyway since it moved in a sqaure)

#publishers for all estimated points for visualization in RVIZ
ep0 = point_stamped_pub("estimated_0", "world", estimated[0])
ep1 = point_stamped_pub("estimated_1", "world", estimated[1])
ep2 = point_stamped_pub("estimated_2", "world", estimated[2])
ep3 = point_stamped_pub("estimated_3", "world", estimated[3])

world_points = projector.map_points_to_world(measured) #convert the measured cartesian points to the world frame

#publishers for these for visualization in RVIZ
mp0 = point_stamped_pub("measured_0", "world", world_points[0])
mp1 = point_stamped_pub("measured_1", "world", world_points[1])
mp2 = point_stamped_pub("measured_2", "world", world_points[2])
mp3 = point_stamped_pub("measured_3", "world", world_points[3])

#functions indicate rotation error between the two sets of points in roll, pitch, yaw
def roll_error(e, m):
	error = (m[0].z + m[1].z + e[2].z + e[3].z) - (e[0].z + e[1].z + m[2].z + m[3].z)
	return error

def pitch_error(e, m):
	error = (m[3].z + m[0].z + e[1].z + e[2].z) - (e[3].z + e[0].z + m[1].z + m[2].z)  	
	return error

def yaw_error(e, m):
	m_yaw = math.atan2((m[0].y -m[3].y), (m[0].x -m[3].x))
	e_yaw = math.atan2((e[0].y -e[3].y), (e[0].x -e[3].x))
	return (m_yaw -e_yaw)

#output the current r,p,y errors
def print_rot_errors(e, m):	
	print("Roll error: ", roll_error(e, m))
	print("Pitch error: ", pitch_error(e, m))
	print("Yaw error: ", yaw_error(e, m))

#publishs all points from both sets for visualization
def pub_calib_points():
		ep0.pub()
		ep1.pub()
		ep2.pub()
		ep3.pub()

		mp0.pub()
		mp1.pub()
		mp2.pub()
		mp3.pub()

#updates all measured points (since they originate from the camera but exist in the world frame their values change as the camera is moved)
def update_calib_points(points):
		mp0.update(points[0])
		mp1.update(points[1])
		mp2.update(points[2])
		mp3.update(points[3])	

#publish the initial points for visualization
for i in range(10):
	update_calib_points(world_points)
	pub_calib_points()
	rate.sleep()

print("Initial rotation error")
print_rot_errors(world_points, estimated)
raw_input("Disable any other camera-world transforms, press Enter to continue...")

#record error indicators for plotting
logr = []
logp = []
logy = []

#for a number of iterations, minimize rotation error between point sets
for i in range(50):
	world_points = projector.map_points_to_world(measured)
	update_calib_points(world_points)
	pub_calib_points()
	y = roll_error(world_points, estimated)
	logr.append(y)
	x = pitch_error(world_points, estimated)
	logp.append(x)
	z = yaw_error(world_points, estimated)
	logy.append(z)
	camera_quat = projector.mov_cam_rot_wrt_world(camera_quat, x, y, z)		
	projector.set_camera_rot(camera_quat)
	rate.sleep()

#output final rotation errors, and show a plot of them
print("Final rotation error")
print_rot_errors(world_points, estimated)

x = np.arange(0,50) 
plt.title("Errors") 
plt.xlabel("Iteration") 
plt.ylabel("Error") 
plt.plot(x,logr, 'r', label='roll error') 
plt.plot(x,logp, 'g', label='pitch error') 
plt.plot(x,logy, 'b', label='yaw error') 
plt.legend(loc="lower right") 
plt.show()

pos_error = [0,0,0] #placeholder for translation errors

world_points = projector.map_points_to_world(measured)
for estimate, measurement in zip(estimated, world_points): #for every point in both sets, compute mean x,y,z translation errors
	print("Est:", estimate)
	print("Mes: ", measurement)
	pos_error[0] += 0.25*(estimate.x -measurement.x)
	pos_error[1] += 0.25*(estimate.y -measurement.y)
	pos_error[2] += 0.25*(estimate.z -measurement.z)

#update the camera transform with the translation errors
camera_trans[0] += pos_error[0]
camera_trans[1] += pos_error[1]
camera_trans[2] += pos_error[2] 

#output calibrated translation, rotation
print("Translation: ")
print(camera_trans)
print("Quaternion: ")
print(camera_quat)

#also ouptput this in XML syntax for use in bringup.launch
print("XML: ")
static_transform = str(camera_trans) +' '  +str(camera_quat)
xml = '<node pkg="tf" type="static_transform_publisher" name="camera_pose_broadcaster" args = "'
xml += static_transform.replace('[', '').replace(',', '').replace(']', '')
xml += ' world camera_link 100" />' 
print(xml)

#do nothing but send the calibrated points for viewing in RVIZ
while(1):
	projector.set_camera_trans(camera_trans)
	world_points = projector.map_points_to_world(measured)
	update_calib_points(world_points)
	pub_calib_points()
	rate.sleep()








