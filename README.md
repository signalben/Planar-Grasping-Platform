# Planar-Grasping-Platform

<p float="left">
<img src="/intro_shot.jpg" width="270" height="350">
<img src="/pregrasp.jpg" width="270" height="350">
</p>

_________________________________________________________________________________
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

_________________________________________________________________________________
## Components
scara_CAD 		- Contains the Solidworks models of the scara arm, gripper, and mounts for the endstops, camera, microcontroller
Arduino_controllers	- Contains the Arduino sketches used for low-level control of the scara arm and gripper
electrical_schematic	- Details electrical components used and is a full wiring diagram
scara_ws		- Is the ROS workspace used for high-level control of the arm
grasp_generation	- Is the implimentation used for generating planar grasps from depth images
_________________________________________________________________________________
## Prerequisites
Ubuntu   version: 16.04
ROS    	 version: kinetic
_________________________________________________________________________________
## Running in demo mode
This repository is already configured to run without a physical robot.
However, an Arduino Mega and an Arduino Nano are required.

For use with a real robot, set real_robot = true in the following files:
/Arduino_controllers/gripper/gripper.ino
/scara_ws/src/scara_grasping/scripts/execute_grasps.py

_________________________________________________________________________________
## Setup

Please refer to the installation instructions in the following files:

/Arduino_controllers/README
/scara_ws/src/README
/grasp_generation/ggcnn-master/README
_________________________________________________________________________________
## Running Manipulator
-after Arduino, ROS, CNN installations, connect Nano to "/dev/ttyUSB0"
-port is specified in ~/scara_ws/src/scara_driver/src/driver.cpp

-In a new terminal:
roscore

-In a new terminal:
rosrun scara_driver driver

-In a new terminal:
rosrun scara_driver user

-In a new terminal:
rostopic echo /update/joint_states

-In the user terminal:
&E
-the driver terminal should now print "Enabled control of arm and gripper")

-In the user terminal:
 #?
-the driver terminal prints the status of GRBL
-this begins with "<alarm|" , indicating that the arm has not been homed and is disabled for safety
-the /update/joint_states topic should echo one new jointstate, as reported by GRBL status message

-Either disable the homing requirement by entering in the user terminal:
 #$X

-Or home a real manipulator to endstops by instead entering:
 #$H

-In the user terminal:
 #?
-The GRBL status message should now begin with <Idle|
-the /update/joint_states topic should echo another jointstate
 
-In the user terminal:
run

-The driver terminal reports "Updating Jointstates from GRBL.."
-the /update/joint_states topic continuously updates at 10hz
-the driver node no longer shows status messages
-updating the jointstates can be disabled by entering 'stop' in the user node

-In a new terminal:
rosrun scara_grasping joint_trajectory_action.py

-In a new terminal:
roslaunch scara_bringup bringup.launch 
  - If no camera is plugged in, use realsense:=0 argument not to run the camera node 
  - To disable the broadcast of a static transform for the camera use argument camera_pose:=0

-RVIZ should open, with the scara manipulator and a motionplanning panel shown on screen
__________________________________________________________________________________
## Veryifying setup

To verify:
- In the 'MotionPlanning' panel, select 'Planning' -> 'Select Goal State' -> 'ready' 
- Then click 'Update', then 'Plan and Execute'
- The model should move from the home position to a new pose, it's position is fedback by GRBL

The arm can also be commanded manually using G-CODE in the user terminal, eg:
#G0 X-30   (moves XJ 30 deg clockwise)
&C150      (sets jaw opening width)
_________________________________________________________________________________
## Grasping

-In a new terminal:
roslaunch scara_grasping cnn.launch

To run the grasping routine-
In a new terminal:
rosrun scara_grasping execute_grasps.py

To only view grasps-
Close execute_grasps.py, in a new terminal: 
rosrun scara_grasping view_grasps.py

-The calibration routine will not work without a real robot
To run the calibration routine for determining the camera pose-
Close execute_grasps.py, view_grasps.py, bringup.launch 

in a new terminal: 
roslaunch scara_bringup bringup.launch camera_pose:=0 

in a new terminal: 
rosrun scara_grasping calibration.py
__________________________________________________________________________________
## Debugging   
Should an unresolvable error occur, restart all terminals and roscore.
This won't lose the manipulators position, the setup commands are listed again below:

roscore
rosrun scara_driver driver
rosrun scara_driver user
 &E
 #$X
 run
rosrun scara_grasping joint_trajectory_action.py
roslaunch scara_bringup bringup.launch 

Common errors:
0 grasps detected   		- Is the RGB-D camera pointed at an object?
Gripper not in view 		- Is an AruCo marker fully visible in /camera/color/image_rect_color?
Failed to get grasp from CNN    - roslaunch scara_grasping cnn.launch

__________________________________________________________________________________

