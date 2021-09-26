#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math 

#function plots a grasp on a 640*480 RGB image for observation - 
#	makes no attempt to compensate for perspective scaling and size of opening width is only approximate
# pose[0] = x, pose[1] = y, pose[3] = angle
def plot_pose(img, pose, width):
	dialX = int(pose[0] + width*math.sin(pose[3])) #draw half a grasp in red from centre of grasp
	dialY = int(pose[1] + width*math.cos(pose[3]))
	dial_start = (int(pose[0]), int(pose[1]))
	dial_end = (dialX, dialY)
	cv2.line(img, dial_start, dial_end, (255, 0, 0), 3)

	dialX = int(pose[0] - width*math.sin(pose[3])) #draw other half in green
	dialY = int(pose[1] - width*math.cos(pose[3]))
	dial_start = (int(pose[0]), int(pose[1]))
	dial_end = (dialX, dialY)
	cv2.line(img, dial_start, dial_end, (0, 255, 0), 3)
	return img #return image with grasp visualization

#class obtains RGB images, and can also get the location of the gripper in those images
class rgbSub:

    def __init__(self):
	self.sub = rospy.Subscriber("/camera/color/image_rect_color", Image, self.callback) #subscribe to rectified colour images
	self.msg = Image
	self.gotImg = False

    def callback(self, msg): #stores RGB image messages, flag to indicate new ones
	self.msg = msg
	self.gotImg = True
	
	#gets single new RGB image
    def get_img(self):
	print("Getting RGB image...")
	while (self.gotImg == False): #do nothing until a new image is recieved
		rospy.sleep(0.05)
	self.gotImg = False
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(self.msg, desired_encoding='passthrough') #convert to numpy array before returning
        return img

#function takes RGB image as argument, finds the location of the gripper in it using a Aruco marker
    def get_gripper(self, img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #does not need colour to identify marker
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000) #the type of marker used is of 4x4 squares
	arucoParameters = aruco.DetectorParameters_create() #no special parameters used
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters) #only interested in corners of marker
	
	if(corners): #if the markers corners are found
		pose = [0,0,0,0] #create empty pose (x,y,z,yaw)
		for corner in corners[0][0]: #corners[0][0] = only marker used
			pose[0] += 0.25*float(corner[0]) #pose x,y is the mean of the four corners x,y
			pose[1] += 0.25*float(corner[1])


		marker = corners[0][0][0] #marker = corner 0, indicates rotation of Fiducial marker 
		angle = math.atan2((pose[0]-marker[0]), (pose[1]-marker[1])) #get angle of this corner relative to centre of Fiducial in radians

		pose[0], pose[1] = round(pose[0]), round(pose[1]) #x, y position of gripper in image must be integers
		pose[3] = angle + 0.88 #offset compensates for the orientation the Fiducial happened to be glued to the gripper in
		
		return pose # returns 2D gripper pose (x,y,z,yaw), z is only a placeholder
	else:
		print("Gripper not in view") #if Aruco fails to detect the Fiducial marker
 		return None 

"""
-References:
For drawing grasps in OpenCV:
https://stackoverflow.com/questions/22252438/draw-a-line-using-an-angle-and-a-point-in-opencv
https://stackoverflow.com/questions/37214036/how-can-i-close-a-cv2-window

Guides for using AruCo markers in openCV:
https://muralimahadeva.medium.com/aruco-markers-usage-in-computer-vision-using-opencv-python-cbdcf6ff5172
https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
"""


