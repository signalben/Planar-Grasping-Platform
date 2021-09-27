import numpy as np
import socket
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy


#ROSlink.py is responsable for recieving depth images, sending generated grasps over sockets
#matches sockets in Planar-Grasping-Platform/scara_ws/src/scara_grasping/scripts/utils/cnn_link.py

#function opens socket, listens for incoming data, decodes depth image
def get_image():
	TCP_IP = 'localhost'
	TCP_PORT = 5001
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind((TCP_IP, TCP_PORT)) 
	s.listen(True)
	print ("Ready to generate grasp")
	conn, addr = s.accept()  	#blocks until cnn_link.py sends data
	length = int(conn.recv(32))  	#read out 32 bytes which contains the length of encoded image
	stringData = conn.recv(length)	#read out encoded image
	s.close()

	missing = length - len(stringData) 		#check if one block contained whole image 

	if (missing > 0):				#if bytes exist in second block
		RemainingStringData = conn.recv(missing)#read them out 
		stringData += RemainingStringData	#add to first block

	data = np.fromstring(stringData, dtype='uint16') #converted encoded image to 1D numpy
	image = cv2.imdecode(data, cv2.IMREAD_ANYDEPTH)  #decode into 300*300 numpy image
	return image 

#function sends grasps as strings to a listening socket from cnn_link.py
def send_grasp(grasp_string):
	TCP_IP = 'localhost'
	TCP_PORT = 5002
	s = socket.socket()
	while(True):	#repeatedly attempts to connect
		try:
			s.connect((TCP_IP, TCP_PORT)) 
			grasp = bytes(grasp_string, 'utf-8') 		#convert grasp string to raw bytes
			length = bytes(str(len(grasp_string)), 'utf-8')	#length of grasp string in bytes
			s.send(length.ljust(16));			#sends length of grasp string first
			s.send(grasp)					#then sends grasp itself
			s.close()
			break
		except:
			pass

	print ("Grasp sent")


"""
References:
	Method used to import OpenCV for Python 3 rather than the 2.7 versioned used by ROS(lines 13-16):
	https://answers.ros.org/question/290660/import-cv2-error-caused-by-ros/
"""




