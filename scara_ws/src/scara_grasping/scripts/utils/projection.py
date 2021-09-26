#!/usr/bin/env python

import pyrealsense2
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import tf
from tf import transformations as ts
from tf import TransformListener
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped
import scipy
from scipy.spatial.transform import Rotation
import math
import numpy as np

#this script helper classes and functions for tf functionality

#function adds a stamp to a point, requires frame string as argument
def add_stamp(inpoint, frame):
	point_stamped = PointStamped()
	point_stamped.header.frame_id = frame
	point_stamped.point = inpoint
	point_stamped.header.stamp = rospy.Time(0)
	return point_stamped

#class used to publish stamped points, takes either pointstamped or pose, publishes to topic string
#stores point for repeated publishing
class point_stamped_pub:

    def __init__(self, topic_name, frame, inpoint):
	self.point_stamped = PointStamped()
	self.point_pub = rospy.Publisher(topic_name, PointStamped, queue_size=1)
	self.point_stamped.header.frame_id = frame
	self.update(inpoint)

    def update(self, inpoint):
	if hasattr(inpoint, 'point'):
		inpoint = inpoint.point
	self.point_stamped.point = inpoint

    def pub(self):
	self.point_stamped.header.stamp = rospy.Time(0)
	self.point_pub.publish(self.point_stamped)

#class used to publish stamped poses, publishes to topic string, takes point and quaternion arguments
#stores pose for repeated publishing, point and quaternion can be updated seperatly
class pose_stamped_pub:

    def __init__(self, topic_name, frame, point, quat):
	self.pose_stamped = PoseStamped()
	self.pose_pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
	self.pose_stamped.header.frame_id = frame	
	self.update_point(point)
	self.update_quat(quat)

    def update_point(self, inpoint):
	if hasattr(inpoint, 'point'):
		inpoint = inpoint.point
	self.pose_stamped.pose.position = inpoint 

    def update_quat(self, quat):
	self.pose_stamped.pose.orientation = quat 

    def pub(self):
	self.pose_stamped.header.stamp = rospy.Time(0)
	self.pose_pub.publish(self.pose_stamped)

#class used to transform from camera space to world 
#move cameras for calibration routine 
class projector:

	def __init__(self):
		self.listener = TransformListener()
		self.grip_pub = rospy.Publisher('/output/grip_point', Point, queue_size=1) #publishes gripper/grasp position for visualization, grasping routine
		self.grasp_pub = rospy.Publisher('/output/grasp_point', Point, queue_size=1)
		self.camera_tf = geometry_msgs.msg.TransformStamped()
		self.cam_broadcaster =  tf2_ros.StaticTransformBroadcaster() #publishes camera pose - conflicts with other static transform boradcasters sending same transform

		self.camera_tf.transform.rotation.x = 0 #initialize camera pose without changing position, orientation 
		self.camera_tf.transform.rotation.y = 0
		self.camera_tf.transform.rotation.z = 0
		self.camera_tf.transform.rotation.w = 1
		self.camera_tf.transform.translation.x = 0
		self.camera_tf.transform.translation.y = 0
		self.camera_tf.transform.translation.z = 0

		intrinsics = pyrealsense2.intrinsics() #construct pyrealsense intrinsics object, values taken from /camera/depth/camera_info topic
		intrinsics.width  = 640
		intrinsics.height = 480
		intrinsics.ppx    = 319.145263671875
		intrinsics.ppy    = 245.34066772460938
		intrinsics.fx     = 611.1975708007812
		intrinsics.fy     = 610.7781982421875
		intrinsics.model  = pyrealsense2.distortion.none #image already rectified
		self.intrinsics   = intrinsics

	#function gets 3D point from a depth image
	#planar_pose[0-1] = x,y pixel in image
	def point_from_image(self, planar_pose, depth_img):
		if(planar_pose is None):
			print("Error- no planar pose")
			return False
		else:
			depth = depth_img[int(planar_pose[1]), int(planar_pose[0])] #depth at x,y position in depth image 
			result = pyrealsense2.rs2_deproject_pixel_to_point(self.intrinsics, [planar_pose[0], planar_pose[1]], depth) #returns point in mm, in Cartesian space
			point = Point(0.001*result[2], 0.014-0.001*result[0], -0.001*result[1]) #convert to metres, flip Z direction, offest by 0.014m as RGB camera offset from depth camera
			print("Camera depth: " + str(depth))
			return point
	
	#looksup the grippers position as determined by tf/robot_state_publisher	
	def get_estimated_gripper_point(self):
				rospy.sleep(1)
				(trans,rot) = self.listener.lookupTransform('world', 'AL', rospy.Time(0))
				point = Point(trans[0], trans[1], trans[2])	
				return point

	#get gripper yaw, blocks until current transform recieved
	#used to correct initial gripper pose,as AL does not have an endstop
	def get_estimated_gripper_yaw(self):
			self.listener.waitForTransform('world', 'AL', rospy.Time(), rospy.Duration(1.0))
			while not rospy.is_shutdown():
				try:
					now = rospy.Time.now()
					self.listener.waitForTransform("world", "AL", now, rospy.Duration(1.0))
					(trans,rot) = self.listener.lookupTransform("world", "AL", now)
					r, p, yaw = ts.euler_from_quaternion(rot) #convert quat to roll, pitch, yaw- only yaw is needed
					return yaw
				except:
					pass
	
	#eul_to_quat is less verbose 
	def eul_to_quat(self, values):
		quat  = tf.transformations.quaternion_from_euler(values[0], values[1], values[2])
		return quat

	#publishes the current camera pose
	def pub_camera_tf(self):
		self.camera_tf.header.stamp = rospy.Time.now()
		self.camera_tf.header.frame_id = "world"
		self.camera_tf.child_frame_id = "camera_link"
		self.cam_broadcaster.sendTransform(self.camera_tf)

	#rotates camera by (roll, pitch, yaw) or quaternion
	def set_camera_rot(self, values):
		
		if (len(values) == 4): #makes sure quaternion is normalized
			length =  math.sqrt(values[0]**2 + values[1]**2 + values[2]**2 + values[3]**2)	
		else:
			values = self.eul_to_quat(values)
			length = 1	
					 
		self.camera_tf.transform.rotation.x = values[0]/length #length normalizes quat, not required for (R,P,Y) 
		self.camera_tf.transform.rotation.y = values[1]/length
		self.camera_tf.transform.rotation.z = values[2]/length
		self.camera_tf.transform.rotation.w = values[3]/length
		self.pub_camera_tf()
 	
	#translates camera by x,y,z
	def set_camera_trans(self, values):
		self.camera_tf.transform.translation.x = values[0]
		self.camera_tf.transform.translation.y = values[1]
		self.camera_tf.transform.translation.z = values[2]
		self.pub_camera_tf()
	"""
	def inverse_quat(self, values):
			quat = [0, 0, 0, 0]
			quat[0] = -values[0]
			quat[1] = -values[1]
			quat[2] = -values[2]
			quat[3] =  values[3]
			return quat
	"""

	#rotates camera link around the axes of the world frame 
	#requires the current camera orientation to update
	def mov_cam_rot_wrt_world(self, camera_quat, x, y, z):
		MQ = ts.quaternion_matrix(camera_quat) #MQ = current camera orientation as homogenous transformation matrix
		
		#MX, MY, MZ = homogenous transformation matrices rotating by specified (x,y,z) values
		s = math.sin(x)
		c = math.cos(x)

		MX =      [[  1,  0,  0,  0  ],
			   [   0,  c,  s,  0  ],
			   [   0, -s,  c,  0  ],
			   [   0,  0,  0,  1  ]]

		s = math.sin(y)
		c = math.cos(y)

		MY =      [[  c,  0, -s,  0  ],
			   [   0,  1,  0,  0  ],
			   [   s,  0,  c,  0  ],
			   [   0,  0,  0,  1  ]]

		s = math.sin(z)
		c = math.cos(z)

		MZ =      [[  c, -s,  0,  0  ],
			   [   s,  c,  0,  0  ],
			   [   0,  0,  1,  0  ],
			   [   0,  0,  0,  1  ]]
		
		#rotate camera sequentially, using MX, MY, MZ
		M = np.dot(MY, MQ)
		M = np.dot(MX, M)
		M = np.dot(MZ, M)
		quat = ts.quaternion_from_matrix(M) #convert new camera orientation to quaternion for use
		return quat

	#converts a point, or list of points, in the camera frame to the world frame
	#adds a stamp to point for later visualization in RVIZ
	def map_points_to_world(self, cam_points):
		
		if(isinstance(cam_points, list)): #returns list if list given
			world_points = []
			for cam_point in cam_points:
				cam_point = add_stamp(cam_point, "camera_link")
				world_point = self.listener.transformPoint("world", cam_point)
				world_points.append(world_point.point)
			return world_points
		else:
			cam_point = add_stamp(cam_points, "camera_link")
			world_point = self.listener.transformPoint("world", cam_point)
			return world_point #returns single point if single point given
	
"""
-References:

Guides for transform broadcasters and listeners
http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20%28Python%29
http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29

Guide for use of the tf transformations module
http://docs.ros.org/en/jade/api/tf/html/python/transformations.html

Converting/normalizing quaternions
https://answers.ros.org/question/316829/how-to-publish-a-pose-in-quaternion/
https://answers.ros.org/question/69754/quaternion-transformations-in-python/

For updating immutable poses
https://stackoverflow.com/questions/38423657/attributeerror-cant-set-attribute-error-in-geometry-msgs-pose-in-ros

Convert pose from one frame to another:
https://answers.ros.org/question/323075/transform-the-coordinate-frame-of-a-pose-from-one-fixed-frame-to-another/
https://stackoverflow.com/questions/65597853/transform-camera-to-world-coordinates

Convert from image space to 3D
https://medium.com/@yasuhirachiba/converting-2d-image-coordinates-to-3d-coordinates-using-ros-intel-realsense-d435-kinect-88621e8e733a
https://towardsdatascience.com/inverse-projection-transformation-c866ccedef1c

"""















		
