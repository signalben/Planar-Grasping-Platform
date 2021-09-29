import os
import glob
import numpy as np
from imageio import imread
import copy

from .grasp_data import GraspDatasetBase
from utils.dataset_processing import grasp, image

"""
Modified from orginal file:
- The get_depth function has been produced which returns floating images for this dataset
- The original get_depth left commented out for training with standard images
- OpenCV import is changed to avoid importing ROS version
"""

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy

class CornellDataset(GraspDatasetBase):
    """
    Dataset wrapper for the Cornell dataset.
    """
    def __init__(self, file_path, start=0.0, end=1.0, ds_rotate=0, **kwargs):
        """
        :param file_path: Cornell Dataset directory.
        :param start: If splitting the dataset, start at this fraction [0,1]
        :param end: If splitting the dataset, finish at this fraction
        :param ds_rotate: If splitting the dataset, rotate the list of items by this fraction first
        :param kwargs: kwargs for GraspDatasetBase
        """
        super(CornellDataset, self).__init__(**kwargs)

        graspf = glob.glob(os.path.join(file_path, '*', 'pcd*cpos.txt'))
        graspf.sort()
        l = len(graspf)
        if l == 0:
            raise FileNotFoundError('No dataset files found. Check path: {}'.format(file_path))

        if ds_rotate:
            graspf = graspf[int(l*ds_rotate):] + graspf[:int(l*ds_rotate)]

        depthf = [f.replace('cpos.txt', 'd.tiff') for f in graspf]
        rgbf = [f.replace('d.tiff', 'r.png') for f in depthf]

        self.grasp_files = graspf[int(l*start):int(l*end)]
        self.depth_files = depthf[int(l*start):int(l*end)]
        self.rgb_files = rgbf[int(l*start):int(l*end)]

    def _get_crop_attrs(self, idx):
        gtbbs = grasp.GraspRectangles.load_from_cornell_file(self.grasp_files[idx])
        center = gtbbs.center
        left = max(0, min(center[1] - self.output_size // 2, 640 - self.output_size))
        top = max(0, min(center[0] - self.output_size // 2, 480 - self.output_size))
        return center, left, top

    def get_gtbb(self, idx, rot=0, zoom=1.0):
        gtbbs = grasp.GraspRectangles.load_from_cornell_file(self.grasp_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        gtbbs.rotate(rot, center)
        gtbbs.offset((-top, -left))
        gtbbs.zoom(zoom, (self.output_size//2, self.output_size//2))
        return gtbbs

        #original get_depth function, uncomment to give standard depth images
    """
    def get_depth(self, idx, rot=0, zoom=1.0):
        depth_img = image.DepthImage.from_tiff(self.depth_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        depth_img.rotate(rot, center)
        depth_img.crop((top, left), (min(480, top + self.output_size), min(640, left + self.output_size)))
        depth_img.normalise()
        depth_img.zoom(zoom)
        depth_img.resize((self.output_size, self.output_size))
        return depth_img.img
    """

        #get_depth converts depth images to 'floating' format before returning them
    def get_depth(self, idx, rot=0, zoom=1.0):
        #image loaded, possibly rotated and cropped as with original get_depth function
        depth_img = image.DepthImage.from_tiff(self.depth_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        depth_img.rotate(rot, center)
        depth_img.crop((top, left), (min(480, top + self.output_size), min(640, left + self.output_size)))

        #estimate background of depth image by taking average of columns from near either edge
        background_column = (depth_img.img[:, 10] + depth_img.img[:, -10])/2

	#subract average background column from entire image 
        for c in range(0, 300):
                depth_img.img[: , c] -= background_column

        #estimate remaining background by taking average of rows from near either edge
        background_row = (depth_img.img[10, : ] + depth_img.img[-10, :])/2

	#subract average background row from entire image       
        for r in range(0, 300):
                depth_img.img[r, : ] -= background_row

	#for detecting object edges, normalize so that Canny edge detector works despite objects varying in scale of depth values
        greyscale = cv2.normalize(depth_img.img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        canny = cv2.Canny(greyscale,50,200)

        #zoom and resize as with original get_depth function
        depth_img.zoom(zoom)
        depth_img.resize((self.output_size, self.output_size))

        #centre image around zero, divide by std deviation so images have similar distributions
        depth_img.img -= depth_img.img.mean()
        depth_img.img /= depth_img.img.std()

        #To find bounding box for object in Canny image:
        thresh = 127 #threshold for detecting canny edge (0/255)
        expand = 20  #expand value for roi around object edges
        box = [0, 299, 0, 299] #start bounding box as full image

	#scan from top down until a pixel value indicates a Canny edge
        for row in range(0, 300):
                scanline = canny[row, :]
                if scanline.max() > thresh:
                        box[0] = (row - expand) #store as top of bounding box
                        break

	#scan from bottom up 
        for row in range(299, 0, -1):
                scanline = canny[row, :]
                if scanline.max() > thresh:
                        box[1] = (row + expand) #store as bottom of bounding box
                        break

	#scan from left to right
        for col in range(0, 300):
                scanline = canny[:, col]
                if scanline.max() > thresh:
                        box[2] = (col - expand) #store as left of bounding box
                        break

	#scan from right to left 
        for col in range(299, 0, -1):
                scanline = canny[:, col]
                if scanline.max() > thresh:
                        box[3] = (col + expand) #store as right of bounding box
                        break

	#low quality of bounding box is indicated by high area
	#if the image is noisy, or the object very large, the bounding box found tends to occupy most of the image area
        low_quality = (box[0]-box[1])*(box[2]-box[3])
        if (low_quality > 25000):        #if this is the case, do not continue extracting object from image
              return depth_img.img/10    #return whole image, /10 to give values in range of -1 to 1

        #otherwise, a suitable bounding box has been found:

	#prevent bounding box from crossing image bounry (limit to 300*300)
        for i in range(0, 4):
                box[i] = max(0, box[i])
                box[i] = min(299, box[i])

	#create roi image using bounding box
        roi = depth_img.img[box[0]:box[1],box[2]:box[3]]

	#perform local background removal using column edges of roi 	
        l_col = copy.deepcopy(roi[:, 1])
        r_col = copy.deepcopy(roi[:, -1])

	#blend from left to right column - so that diagonal background gradients are removed       
        for c in range(0, roi.shape[1]):
                ratio = float(c/roi.shape[1])
                roi[: , c] -= (1.0-ratio)*l_col
                roi[: , c] -= ratio*r_col 

	#repeat process for rows of roi
        t_row = copy.deepcopy(roi[1,  : ])
        b_row = copy.deepcopy(roi[-1, : ])
       
        for r in range(0, roi.shape[0]):
                ratio = float(r/roi.shape[0])
                roi[r, : ] -= (1.0-ratio)*t_row
                roi[r, : ] -= ratio*b_row 

        #roi should now be very close to zero for all pixels other than the object

        #create empty image of all zeros 
        depth = np.zeros((300,300), dtype =float)

        #place roi into empty image at the same location it was taken from originally
        depth[box[0]:box[1],box[2]:box[3]] = roi

        return depth/10 #return full image, /10 to give values in range of -1 to 1

    def get_rgb(self, idx, rot=0, zoom=1.0, normalise=True):
        rgb_img = image.Image.from_file(self.rgb_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        rgb_img.rotate(rot, center)
        rgb_img.crop((top, left), (min(480, top + self.output_size), min(640, left + self.output_size)))
        rgb_img.zoom(zoom)
        rgb_img.resize((self.output_size, self.output_size))
        if normalise:
            rgb_img.normalise()
            rgb_img.img = rgb_img.img.transpose((2, 0, 1))
        return rgb_img.img

"""
References:
	Method used to import OpenCV for Python 3 rather than the 2.7 versioned used by ROS(lines 10-13):
	https://answers.ros.org/question/290660/import-cv2-error-caused-by-ros/
"""
