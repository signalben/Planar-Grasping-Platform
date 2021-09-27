# Planar-Grasping-Platform

<p float="left">
<img src="/intro_shot.jpg" width="270" height="350">
<img src="/pregrasp.jpg" width="270" height="350">
</p>

## Demonstration videos

Successful grasps:  
https://youtu.be/Sxl_OcmL-3o  
https://youtu.be/Iijbrh8ghpE  
https://youtu.be/rKQw1soyyW4  

Failed grasps:  
https://youtu.be/VdM2XRzhyX0  
https://youtu.be/__iQ3P3LJT0  
https://youtu.be/dziLtvFLTpc  

Visualizations:  
https://youtu.be/MQvtPGrG9rY  
https://youtu.be/M_flOcwguvU

## Components
scara_CAD 		- Contains the Solidworks models of the scara arm, gripper, and mounts for the endstops, camera, microcontroller
Arduino_controllers	- Contains the Arduino sketches used for low-level control of the scara arm and gripper
electrical_schematic	- Details electrical components used and is a full wiring diagram
scara_ws		- Is the ROS workspace used for high-level control of the arm
grasp_generation	- Is the implimentation used for generating planar grasps from depth images

## Prerequisites
Ubuntu   version: 16.04.7 LTS  
ROS    	 version: kinetic  
_________________________________________________________________________________
## Installation
$ cd Planar-Grasping-Platform/scara_ws/
$ catkin_make
$ sudo gedit ~/.bashrc
---Add the fullpath of scara_ws/devel/setup.bash to bashrc:
$ source /home/"YOUR_USERNAME"/Planar-Grasping-Platform/scara_ws/devel/setup.bash


source ~/anaconda3/etc/profile.d/conda.sh
__________________________________________________________________________________
## Running
__________________________________________________________________________________  
## Testing and debugging    
__________________________________________________________________________________
## Notes
