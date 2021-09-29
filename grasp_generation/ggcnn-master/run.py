import logging
import torch.utils.data
from models.common import post_process_output
from utils.dataset_processing import evaluation, grasp
from utils.data import get_dataset
from ROSlink import get_image, send_grasp
from PIL import Image
import numpy as np
logging.basicConfig(level=logging.INFO)
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from utils.dataset_processing.grasp import detect_grasps
import cv2
import math
import sys

#this script is used to produce grasps from depth images, main loop for grasp generation on line 175
#allows choice of models, line 46
#communication with ROS achived using get_image, send_grasp from ROSlink.py
#allows the capture of an empty work surface as background for replacement of background in depth images, line 90
#allows viewing of depth images and grasp map images, lines 204 and 228

get_background_calibration = 0 	#use 1 to capture empty workspace as background, saves to file, then exits 
background_removal = 1		#use 1 to remove saved background during operation
show_output = 0			#show output grasp maps from model 
show_input = 0			#show input depth image
m_select = 4			#select which model to use
adjust_grasp_position = 0	#apply shifting of grasp positon based on distance from centre of image, object height (not used by default)
full_dataset_available = 0

#show single channel image using colormap
def show_colormap(windows_name, image, normalize):		
			image = (image.astype('float32'))
			if(normalize == True):
				image = image/image.max()
				image = 255*image 
			image = image.astype(np.uint8)
			image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
			cv2.imshow(windows_name, image)
			result = cv2.waitKey(10)

#show single channel image as greyscale
def show_greyscale(windows_name, image):
			cv2.imshow(windows_name, image)
			result = cv2.waitKey(10)

if(m_select == 1):
	model = torch.load("trained_models/1_G1_80_77")	#GGCNN1 80/100 epochs
	floating = 0

elif(m_select == 2):
	model = torch.load("trained_models/2_G2_22_86")	#GGCNN2 22/100 epochs
	floating = 0

elif(m_select == 3):
	model = torch.load("trained_models/3_G2_50_79_F")	#GGCNN2 50/100 epochs floating normalized images
	floating = 1

elif(m_select == 4):
	model = torch.load("trained_models/4_G2B_35_80_F")	#GGCNN2 35/100 epochs floating normalized images, batch normalization
	floating = 1

elif(m_select == 5):
	model = torch.load("trained_models/5_G2B_82_89") 	#GGCNN2 82 epochs, batchnorm
	floating = 0

elif(m_select == 6):
	model = torch.load("pretrained_models/ggcnn_epoch_23_cornell")	#GGCNN1 23 epochs, pretrained
	floating = 0

elif(m_select == 7):
	model = torch.load("pretrained_models/epoch_50_cornell") 	#GGCNN2 50 epochs, pretrained
	floating = 0

elif(m_select == 8):
	model = torch.load("trained_models/8_G2_93_82") 	#GGCNN2 93 epochs
	floating = 0

#use gpu for running model
device = torch.device("cuda:0")


if (full_dataset_available ==  1):
	#load Cornell dataset as object (requires ggcnn-master/utils/dataset_processing/generate_cornell_depth.py to have been run to convert pointcloud files to tiff images)
	Dataset = get_dataset("cornell")
	cornell_data = Dataset("home/YOUR_USERNAME/Planar-Grasping-Platform/grasp_generation/ggcnn-master/cornell")
	idx = 221 #coffee cup--221
	cornell_img = cornell_data.get_depth(idx) #load single depth image from Cornell dataset

else:	
	cornell_img = np.loadtxt('cornell_sample.txt', dtype=float)  #coffee cup--221
	

#routine to capture background image
if(get_background_calibration == True):
	scale_factor = 1
	error_tolerance = 0.01

	c_curve = 0.5*(cornell_img[:,50] + cornell_img[:,250]) #column average from edges of image
	c_background = []
	for i in range(300):		#copy column to create 300*300 background image for dataset
		c_background.append(c_curve)
	c_background = np.flipud(np.rot90(np.array(c_background)))

	np.savetxt('cornell_background.txt', c_background, fmt='%f') #save dataset background
	done = False    #indicates if calibration has already been performed, replace with counter for repeated calibrations
			#program structured to allow repeated iterative calibration, but was found to be only need a single iteration

	while(True):
		depth_img = get_image()	#blocks until depth image recieved on socket from ROS

		if(depth_img is not None):
			print("Mean" , np.mean(depth_img))
			depth_img[0:1, : ] = depth_img[2:3, : ]	  #bottom 2 rows from camera have 'dead pixels', replace with next row up
			depth_img[1:2, : ] = depth_img[2:3, : ]
			depth_img = (depth_img.astype('float32'))

			print("Scale factor")
			print(scale_factor)
			depth_img = (depth_img - np.mean(depth_img))*scale_factor #zero mean and scale depth image
			
			#produce mean column for entire depth image
			gradient = depth_img.copy()
			for i in range(300):
				if(i==0):
					r_curve = gradient[:,i]
				else:
					r_curve += gradient[:,i]
			r_curve = r_curve/300 

			#plot both background curves together before calibration
			x = np.arange(0,300)  #axis to plot  curve against
			plt.ylabel("Pixel value") 
			plt.xlabel("Pixel row") 
			plt.plot(x,c_curve, 'g', label='Dataset background') 
			plt.plot(x,r_curve, 'r', label='Real-world background') 
			plt.legend(loc="upper right") 
			plt.show()
			
			#if the background calibration has already been performed
			if(done == True):		
				
				#save the background image and scale factor for use when running the model	
				np.savetxt('depth_background.txt', depth_img, fmt='%f')
				print("depth_background  min :" + str(depth_img.min()) + " Max: " + str(depth_img.max()) + " Mean: " + str(depth_img.mean()) + " Std: " + str(depth_img.std()) )
				np.savetxt('scale_factor.txt', np.array([scale_factor]), fmt='%f')

				#plot both background curves together after calibration
				f = plt.figure()
				f.add_subplot(2,2, 1)
				plt.imshow(depth_img, cmap = 'gray')
				f.add_subplot(2,2, 2)
				plt.imshow(c_background, cmap = 'gray')
				plt.show(block=True)
				plt.show()
				sys.exit()	#calibration done.

			#if the background calibration has not already been performed
			print("Background curves")
			#calulate correction constant for aligning depth image background curve with cornell image background curve
			#no need to offset the curve - they both already have zero mean
			error = 0.5*(c_curve[0]/r_curve[0]) + 0.5*(c_curve[-1]/r_curve[-1]) 
			print("error =", error) 
			scale_factor = scale_factor*error #apply correction to scale factor
			print("Scale factor")
			print(scale_factor)
			done = True #next loop will now plot calibrated curves before saving calibration, showing background images, exiting

#background calibration done previously - load background images from file
c_background = np.loadtxt('cornell_background.txt', dtype=float)
r_background = np.loadtxt('depth_background.txt', dtype=float)
scale_factor = float(np.loadtxt('scale_factor.txt', dtype=float))

#main loop for grasp generation
while(True):
	with torch.no_grad(): #only using forward pass of model
		depth_img = get_image()	#blocks until depth image recieved on socket from ROS
		if(depth_img is not None):

			print("Got depth image")
			depth_img[0:1, : ] = depth_img[2:3, : ]	#bottom 2 rows from camera have 'dead pixels', replace with next row up
			depth_img[1:2, : ] = depth_img[2:3, : ]
			
			depth_img = depth_img.astype(np.float32)

			if(background_removal):	#if removing depth image background, zero mean and scale to match dataset background
				depth_img = (depth_img - np.mean(depth_img))*scale_factor
				depth_img = depth_img - r_background	#subtract background image previously stored

			else:
				depth_img = (depth_img - np.mean(depth_img))/255 #otherwise zero mean and scale to roughly -1 to 1

			object_height = -depth_img.min() #minimum value indicates object height

			if(floating == 0):	
				depth_img += c_background #if the model does not expect floating images, add the Cornell background

			if(floating == 1):
				depth_img -= depth_img.mean() #if the model does expect floating images, 
				depth_img /= depth_img.std()  #then centre, normalize, and scale similar to training of model
				depth_img /= 10

			if(show_input):	#Display depth image, and a dataset depth image, for comparison						
				print("Depth min :" + str(depth_img.min()) + " Max: " + str(depth_img.max()) + " Mean: " + str(depth_img.mean()) + " Std: " + str(depth_img.std()) )
				f = plt.figure()
				f.add_subplot(2,2, 1)
				plt.imshow(depth_img, cmap = 'gray')

				print("Dataset min :" + str(cornell_img.min()) + " Max: " + str(cornell_img.max()) + " Mean: " + str(cornell_img.mean()) + " Std: " + str(cornell_img.std()) )
				f.add_subplot(2,2, 3)
				plt.imshow(cornell_img, cmap = 'gray')
				plt.show(block=True)
				plt.show()

			np_input = np.expand_dims(depth_img, 0)	#expand image to 1*300*300
			np_input = np.expand_dims(np_input, 0)  #expand image to 1*1*300*300 (model expects batch of one channel 2D images)
			x = torch.from_numpy(np_input.astype(np.float32)) #convert to PyTorch tensor
				
			x_gpu = x.to(device)	#put this tensor on the GPU
			prediction = model(x_gpu)	#forward pass generate grasp maps
			#converts horizontal, vertical grasp map to angle grasp map, applies slight blurring
			q_img, ang_img, width_img = post_process_output(prediction[0], prediction[1], prediction[2], prediction[3])

			if(show_output): #view grasp maps 
				show_greyscale("quality", q_img)
				#show_greyscale("Quality", q_img)
				#show_colormap("Angle", ang_img, True)
				#show_colormap("Width", width_img, True)
				print("Max grasp quality: ", q_img.max())

			#detects one grasp by default
			gs = detect_grasps(q_img, ang_img, width_img)

			if(len(gs) == 1): 	#less or more than one grasp is an error condition
				grasp = [gs[0].center[0], gs[0].center[1], gs[0].angle, gs[0].width]

				if(adjust_grasp_position): #used to correct for model assumptions of perspective, not found benificial  
					disty = (300 - grasp[0])
					grasp[0] -= int(0.3*(object_height*disty))

					distx = (150 - grasp[1])
					grasp[1] -= int(0.3*(object_height*distx))

					grasp[0] = max(min(299, grasp[0]), 0) 
					grasp[1] = max(min(299, grasp[1]), 0)
		
				#get the local high point around grasp			
				rsize = 22 #size of sqaure to consider local
				bounds = [grasp[0] - rsize, grasp[0] + rsize, grasp[1] - rsize, grasp[1] + rsize] #local bounding box
				
				#bounding box cannot cross image edges
				for i, bound in enumerate(bounds):	
					bounds[i] = max(min(bound, 300), 0)

				roi = depth_img[bounds[0]:bounds[1],bounds[2]:bounds[3]]	#extract local roi
				high_x, high_y = np.unravel_index(roi.argmin(), roi.shape)	#coordinates of local high point (min distance to camera)
				grasp.append(high_x + bounds[0])				#send these coordinates at end of grasp
				grasp.append(high_y + bounds[2])
		

				grasp_string = str(grasp)	#convert grasp to string
				print(grasp_string)	
				send_grasp(grasp_string)	#send string back to ROS over socket	

			#lets the user know if no grasps, or multiple grasps, are given
			elif(len(gs) > 1):
				print(str(len(gs)) + " grasps detected - None will be used")
			else:
				print(str(len(gs)) + " grasps detected")






			











