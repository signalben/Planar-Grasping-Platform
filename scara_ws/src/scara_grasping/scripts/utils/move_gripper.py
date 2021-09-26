#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf import transformations as ts

from sensor_msgs.msg	 import JointState
from std_msgs.msg import Header
from std_msgs.msg	 import Char
from std_msgs.msg	 import String

#this class allows control of the gripper via PID or clasp command
class move_gripper:

	#single char, sent by driver node reports gripper status U=unknown, T= PID target reached, C = clasping sucessful, F = PID/closure failed 
    def status_callback(self, status):
	self.status = chr(status.data)

    def __init__(self):
	self.joint_pub  = rospy.Publisher('/target_joint_states', JointState, queue_size=1) #publishes desired gripper position
	self.direct_pub = rospy.Publisher('/scara_write', String, queue_size=1) #write clasp command to serial through driver node
	
	self.status = 'U' # initialize with unknown gripper status
	self.reset_pub  = rospy.Publisher('/grip_status', Char, queue_size=1) #set gripper status (prevents previous results from having an effect) 
	self.status_sub = rospy.Subscriber('/grip_status', Char, self.status_callback) #get gripper status (published by driver node)

    def wait_for_result(self): #blocks until gripper status is obtained

	reset = Char() #rospy char message
	reset.data = 85 #char of rospy char = 'U', unknown
	self.reset_pub.publish(reset) #set status as unknown

	while(self.status != 'U'): #block until status is U
		rospy.sleep(0.01)

	while(self.status == 'U'): #block until status is changed by the driver node
		rospy.sleep(0.01)

	if  (self.status == "T"): #success, return true
		print("Gripper target reached")
		return True

	if(self.status == "C"): #success, return true
		print("Closure sucessful")
		return True

	else:
		print("Gripper failed") #gripper reported failure of PID/clasping
		return False

	#allows setting of gripper opening width in mm, uses PID onboard the gripper
    def set(self, millimetres):

	joint_state = JointState() #produce message with header
	joint_state.header = Header()
	joint_state.header.stamp = rospy.Time.now()
	joint_state.name = ["G0"] #only specify G0 target position (G1 is mechanically mirrored)
	jval = float(millimetres)/2000 #/2000 as opening width is between G0 and G1
	joint_state.position = [jval]
	self.joint_pub.publish(joint_state) #publish target G0 gripper position
	
	print("Set gripper request...")
	result = self.wait_for_result() #block unitl position acheived


	#allows setting of constant PWM value, until gripper detects that motion is blocked by an object 
    def clasp(self, pwmval):

	gripper_cmd = String()
	gripper_cmd.data = "&P" + str(pwmval) #does not use Jointstates, publishes direct message to gripper prefixed with '&P'
	self.direct_pub.publish(gripper_cmd)

	print("Clasp gripper request...")
	result = self.wait_for_result()
	#returns false if the gripper detects no object

"""
-References:

Explains jointstate structure:
http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html

"""








