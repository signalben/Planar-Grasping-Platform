#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math

#class is responsable for getting depth images, sending them to the CNN, returning grasps from the CNN
#matches sockets in Planar-Grasping-Platform/grasp_generation/ggcnn-master/ROSlink.py
class depthSub:

    def __init__(self, full_FOV):
	self.sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback) #subscribe to rectified depth images
	self.msg = Image
	self.gotImg = False
	self.full_FOV = full_FOV #use either 480*480 pixels or 300*300 pixels of depth image

    def callback(self, msg): #stores depth image, keeps flag to indicate that it has a new one 
	self.msg = msg
	self.gotImg = True
	
	#returns a depth image
    def get_img(self):
	num_samples = 3 #allows several depth images to be averaged to reduce noise
	print("Waiting for depth image...")

	for i in range(num_samples):
		while (self.gotImg == False): #do nothing until new depth image recieved
			pass
		self.gotImg = False
		bridge = CvBridge()
		img = bridge.imgmsg_to_cv2(self.msg, desired_encoding='passthrough') #convert ROS image message to numpy array

		ret, mask = cv2.threshold(img, 0, 1, cv2.THRESH_BINARY) #mask is all pixels with a valid depth reading (invalid have a zero value)
		if (i == 0): #if first image recieved
			avg_img = img 
			avg_mask = mask
		else:
			avg_img  = avg_img + img #for subsequent images, add them together
			avg_mask = cv2.bitwise_and(avg_mask, mask) #and masks so that only pixels valid in images are considered valid
	
	
	avg_img = avg_img/num_samples #blend of all images
	black_img = np.zeros((300, 300), np.uint16) #empty image
	avg_img *= avg_mask #set all pixels of averaged image to 0 if the mask lists them as invalid
	ret, mask = cv2.threshold(avg_img, 0, 10, cv2.THRESH_BINARY_INV) #create new mask indicating invalid pixels 
	mask = np.uint8(mask)  
	avg_img = cv2.inpaint(avg_img,mask,3,cv2.INPAINT_TELEA) #inpaint invalid pixels to obtain final depth image
        return avg_img

	#function sends depth image via sockets to CNN
    def send_depth_image(self, img):
	print("Sending depth image for interpretation by GGCNN2...")

	if(self.full_FOV == True): #if using a 480*480 area, resize to 300*300 
		img = img[:, 80:-80]
		img = cv2.resize(img, (300, 300), interpolation = cv2.INTER_AREA)
	else:
		img = img[ 90: -90, 170:-170] #if using central portion of 640*480 image, take central 300*300 portion 


	img = img.astype(np.uint16) #should already be uint16

	result, imgencode = cv2.imencode('.tiff', img) #encode as tiff, should be lossless
	data = np.array(imgencode) #convert to numpy array
	stringData = data.tostring() #convert to string

	TCP_IP = 'localhost' #setup socket to send encoded image
	TCP_PORT = 5001
	try:			#try to connect to socket in Anaconda environment
		s = socket.socket()
		s.connect((TCP_IP, TCP_PORT)) 	
		s.send(str(len(stringData)).ljust(32)) #send the length of the encoded image as exactly 32 bytes
		s.send(stringData) #send the encoded image
		print("OK")
	except:
		print("GGCNN2 failed to connect")

	#function recieves grasp from CNN in Anaconda environment, takes a depth image as an argument
    def get_grasp(self, img):

	self.send_depth_image(img) #send the image to CNN

	TCP_IP = 'localhost'
	TCP_PORT = 5002 #recieves using a different socket
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #setup socket to listen
	s.settimeout(0.2) #allow 0.2 seconds before assuming an error
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind((TCP_IP, TCP_PORT)) 
	s.listen(True)
	print ("Waiting for grasp...")
	try:
		conn, addr = s.accept() #try to recieve incoming data
		length = conn.recv(16)  #16 bytes used to indicate length of grasp string
		s.settimeout(None)
		gsp = conn.recv(int(length)) #recieve grasp as a string (gsp)
		s.close()
	except:
		print("Failed to get grasp from CNN")
		return None, None

	print("Grasp received")
	print(gsp) #print grasp string for observation
	gsp = gsp[1:-1] #remove [ and ]
	gsp = gsp.replace(',', '') #remove ,
	gsp = gsp.split() #list of strings 
	
	grasp = [float(gsp[1]), float(gsp[0]), 0, float(gsp[2]), float(gsp[3])] #cast strings as x,y,z,w,width floats
	high_point = [float(gsp[5]), float(gsp[4])] #indicates the local high point around grasp for z axis positioning, not used by default

	if(self.full_FOV == True): #place grasp, and high point, within the full 640*480 image
		print("Full FOV")
		grasp[1] = 1.6*grasp[1] #undo earlier scaling of image size
		grasp[0] = 90 + 1.6*grasp[0] 
		high_point[0] = 80 + (1.6*high_point[0]) #undo earlier scaling of image size
		high_point[1] =       1.6*high_point[1]

	else:
		print("Zoom FOV on centre 300x300 section") #place grasp, and high point, within the full 640*480 image
		grasp[0] += 170
		grasp[1] += 90
		high_point[0] += 170
		high_point[1] += 90

	grasp[4] = min(grasp[4], 80)    #limit jaw opening width to 80mm
	grasp[3] += math.pi/2		#rotate 90deg to align grasp with axis of jaws

	return grasp, high_point

"""
-References:

Code used to send image VIA TCP sockets:
https://stackoverflow.com/questions/20820602/image-send-via-tcp

Code used for subscription of images:
http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
"""




	








