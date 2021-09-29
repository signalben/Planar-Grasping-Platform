#!/usr/bin/env python
import rospy
import time
import thread
import actionlib

from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from scara_grasping.msg  import * 
from sensor_msgs.msg	 import JointState
from std_msgs.msg	 import Char
from std_msgs.msg	 import Float32


#this script provides a controller to moveit in the form of a joint trajectory action sevrer

#feedback sub captures the latest joint states, and also the total joint error calcuated by the driver node
class feedback_sub():

	def error_callback(self, msg):
		self.error = float(msg.data)
		self.recent = True

	def joint_states_callback(self, msg):
		self.joint_states = msg

	def __init__(self):
		self.current_joint_states = JointState()
		self.feedback_sub = rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
		self.error_sub = rospy.Subscriber('/total_joint_error', Float32, self.error_callback)


#class accepts goal from moveit, gives feeback, result
class JTAction(object):

	def execute_cb(self, goal): #when goal recieved from moveit
			trajectory_point = goal.trajectory.points[-1].positions #take the last waypoint specified as the desired position
			print("Goal recieved")
			print(trajectory_point)

			target_joint_states = JointState() #construct a joint state message containing the desired position
			joint_names = goal.trajectory.joint_names		
			target_joint_states.position = trajectory_point
			target_joint_states.name = joint_names

			self.joint_pub.publish(target_joint_states) #publish this desired joint state as a target for the driver node to acheive
			print("Sent target to GRBL")

			feedback.recent = False
			print("Waiting for feedback")
			while(feedback.recent == False):	#block until new total_joint_error is reported by driver node (should be non-zero, as new target just set) 
				rospy.sleep(0.01)
			
			print("Start position error: " + str(feedback.error)) #print total joint error at start of move, feedback start position to moveit
			self.update_feedback()
			while(feedback.error > 0.1):	#block until error is below 0.1	
				rospy.sleep(0.01)

			print("End position error: " + str(feedback.error)) #feedback new position to moveit, mark as action succeeded
			self.update_feedback()
			self._as.set_succeeded(self._result)
			print("Complete")

	#initialize action sever, allow publishing of target joint states
	def __init__(self):
		self.joint_pub = rospy.Publisher('/target_joint_states', JointState, queue_size=1)
		self._as = actionlib.SimpleActionServer( '/scara/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
		self._fdbk = FollowJointTrajectoryFeedback()
		self._result = FollowJointTrajectoryResult()
		self._as.start()

	#gives feedback message to moveit, uses latest joint states captured by feedback_sub
	def update_feedback(self):

		self._fdbk.joint_names = feedback.joint_states.name
		self._fdbk.actual.positions = feedback.joint_states.position
		self._fdbk.header.stamp = rospy.Time.now()
		self._as.publish_feedback(self._fdbk)
	

#main only intializes action server and feedback sub, all further events driven by callbacks
if __name__ == '__main__':
	rospy.init_node('scara_action_server')
	feedback = feedback_sub()
	print("Action server running...")
	JTAction()
	rospy.spin()

"""
-References:
Adapted from the Baxster JointTrajectoryAction controller:
https://github.com/RethinkRobotics/baxter_interface/tree/master/src/joint_trajectory_action
https://github.com/ros-industrial/robot_movement_interface/blob/master/ur_driver/scripts/joint_trajectory_action.py
"""

