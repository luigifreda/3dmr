#!/usr/bin/env python
## @file
# Set of helper tools for the teleoperation of the NIFTi robot
#
# Two things are implemented:
# - ScanningService: handle the motion of the rolling laser;
# - FlipperPosture: handle flipper postures.
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from rospy.rostime import get_time
from std_msgs.msg import Bool, Float64, Int32
from sensor_msgs.msg import PointCloud2
from nifti_robot_driver_msgs.msg import RobotStatusStamped, FlippersState,\
		FlippersStateStamped
#import tf
from sensor_msgs.msg import JointState

import actionlib
from actionlib.server_goal_handle import ServerGoalHandle
from actionlib_msgs.msg import GoalStatus
from nifti_teleop.msg import *

from math import atan2, pi, radians, floor, sin

class FlipperPosture(object):
	'''Handle the configuration of the flippers.
	'''
	postures={
		# 0: flat
		0: (0, 0, 0, 0),
		# 1: drive
		1: (radians(-165), radians(-165), radians(115), radians(115)),
		# 2: climb forward
		2: (radians(-45), radians(-45), radians(10), radians(10)),
		# 3: climb backward
		3: (radians(-10), radians(-10), radians(45), radians(45)),
		# 4: convex edge
		4: (radians(40), radians(40), radians(-40), radians(-40)),
		# 5: bestInit
		5: (radians(-135), radians(-135), radians(135), radians(135))}
		

	def __init__(self):
		self.flippers_offset = FlippersState()
		self.flippers_pub = rospy.Publisher('flippers_cmd', FlippersState, queue_size=10)
		self.posture_pub = rospy.Publisher('posture', Int32, queue_size=1)
		rospy.Subscriber('flippers_state', FlippersStateStamped,
				self.flippers_cb, queue_size=10)
		rospy.Subscriber('posture_cmd', Int32, self.cmd_cb, queue_size=10)

	def cmd_cb(self, posture):
		pid = posture.data
#		rospy.logdebug("NTH - Received posture command: %d"%pid)
		try:
			fl, fr, rl, rr = self.postures[pid]
			fs = FlippersState()
			fs.frontLeft = fl + self.flippers_offset.frontLeft
			fs.frontRight = fr + self.flippers_offset.frontRight
			fs.rearLeft = rl + self.flippers_offset.rearLeft
			fs.rearRight = rr + self.flippers_offset.rearRight
			self.flippers_pub.publish(fs)
		except IndexError:
			rospy.logerr("NTH - Unknow posture id: %d"%pid)
	
	def flippers_cb(self, flippers):
		fl = flippers.frontLeft
		fr = flippers.frontRight
		rl = flippers.rearLeft
		rr = flippers.rearRight
		self.flippers_offset.frontLeft = 2*pi*floor((fl+1.5*pi)/(2*pi))
		self.flippers_offset.frontRight = 2*pi*floor((fr+1.5*pi)/(2*pi))
		self.flippers_offset.rearLeft = 2*pi*floor((rl+0.5*pi)/(2*pi))
		self.flippers_offset.rearRight = 2*pi*floor((rr+0.5*pi)/(2*pi))
		tol = sin(radians(5))
		ret_pid = -1
		for pid, (pfl, pfr, prl, prr) in self.postures.items():
			if abs(sin(fl-pfl))<=tol and abs(sin(fr-pfr))<=tol\
					and abs(sin(rl-prl))<=tol and abs(sin(rr-prr))<=tol:
				ret_pid = pid
				break
		self.posture_pub.publish(ret_pid)


## ScanningService class
class ScanningService(object):
	'''Handle the motion of the rolling laser for taking a single 3D scan.
	'''

	## Instantiate the ScanningService class
	def __init__(self):
		## scanning speed command publisher
		self.scanning_speed_pub = rospy.Publisher('scanning_speed_cmd', Float64, queue_size=1)
		## laser center command publisher
		self.laser_center_pub = rospy.Publisher('laser_center', Bool, queue_size=1)
		## robot status subscriber (to get the current speed of the laser)
		rospy.Subscriber('robot_status', RobotStatusStamped, self.status_cb, queue_size=10)
		## scanning_once subscriber
		rospy.Subscriber('scanning_once', Float64, self.scanning_once_cb, queue_size=1)
		## point cloud control publisher
		self.ptcld_ctrl_pub = rospy.Publisher('pointcloud_control', Bool, queue_size=1)
		## scanning state
		self.scanning_state = ScanningFeedback.READY
		## last goal time set, to avoid race condition
		self.last_goal_time = get_time()
		## action server
		self.goal = ServerGoalHandle()
		self.action_server = actionlib.ActionServer('ScanningOnceAS',
				ScanningAction, self.goal_cb, auto_start=False)
		self.action_server.start()
		## action client
		self.action_client = actionlib.SimpleActionClient('ScanningOnceAS',
				ScanningAction)

		self.last_angle = 0
		self.last_direction = 0
		rospy.Subscriber('joint_states', JointState, self.joint_states_cb, queue_size=10)
		# tf listener for laser scanner angle
		#self.tf_listener = tf.TransformListener()

	## joint state callback to get the current position of the laser
	def joint_states_cb(self, joint_states):
		if not "laser_j" in joint_states.name:
			return

		laser_idx = joint_states.name.index('laser_j')
		angle = joint_states.position[laser_idx]
		if angle>self.last_angle:
			direction = 1
		elif angle<self.last_angle:
			direction = -1
		else:
			direction = 0
		if self.scanning_speed*direction*self.last_direction < 0 and\
				abs(angle)>pi/3:	# TODO might be problematic
			# End of swipe event
			rospy.logdebug("NTH - End of swipe")
			if self.scanning_state == ScanningFeedback.WAITING_FOR_FIRST_SWIPE:
				# no end of swipe received before
				rospy.logdebug("NTH - Detected first end of swipe: waiting for a \
second one and activating point cloud publication.")
				self.ptcld_ctrl_pub.publish(True)
				self.scanning_state = ScanningFeedback.WAITING_FOR_COMPLETE_SWIPE
				self.goal.publish_feedback(ScanningFeedback(self.scanning_state))
			elif self.scanning_state == ScanningFeedback.WAITING_FOR_COMPLETE_SWIPE:
				# first end of swipe received before
				rospy.logdebug("NTH - Detected second end of swipe: stopping laser and \
centering it.")
				self.scanning_state = ScanningFeedback.READY
				#self.scanning_speed_pub.publish(0.0) # may be unnecessary
				self.laser_center_pub.publish(True)
				self.goal.publish_feedback(ScanningFeedback(self.scanning_state))
				self.goal.set_succeeded(ScanningResult(ScanningResult.SUCCESS),
						"Scan succeeded")
				rospy.loginfo("NTH - Scan ended")
			else:
				rospy.logdebug("NTH - end of swipe received and ignored.")
		self.last_direction = direction
		self.last_angle = angle
	
	
	## robot status callback to get the current speed of the laser
	def status_cb(self, robot_status):
		self.scanning_speed = robot_status.scanning_speed
		if (self.scanning_speed == 0.0) and \
				(get_time() - self.last_goal_time>0.5):	
			# if not moving, updating state
			rospy.logdebug('NTH - Laser speed 0: setting state to "Not scanning".')
			if self.scanning_state != ScanningFeedback.READY:
				self.scanning_state = ScanningFeedback.READY
				self.goal.set_succeeded(ScanningResult(ScanningResult.ERROR),
						"Aborting as laser is not moving")
				self.goal.publish_feedback(ScanningFeedback(self.scanning_state))


	def cancel_cb(self, goal):
		if goal==self.goal:
			self.stop_scanning()	# Temporary

	## callback when a goal is received
	def goal_cb(self, goal):
		if self.goal.get_goal() and \
				self.goal.get_goal_status().status == GoalStatus.ACTIVE:
			goal.set_rejected(ScanningResult(ScanningResult.WARNING),
				"Can only do one scan at a time")
			return
		scanning_goal = goal.get_goal()
		if scanning_goal.action == ScanningGoal.START_SCANNING:
			if self.scanning_state == ScanningFeedback.READY and\
					self.scanning_speed==0:
				self.goal = goal
				self.goal.set_accepted("Scan accepted")
				self.start_scanning(scanning_goal.speed)
			else:
				goal.set_rejected(ScanningResult(ScanningResult.ERROR),
						"Already scanning")
		elif scanning_goal.action == ScanningGoal.STOP_SCANNING:
			self.goal = goal
			self.goal.set_accepted("Stop accepted")
			self.stop_scanning()
		else:
			goal.set_rejected(ScanningResult(ScanningResult.ERROR),
					"Unknown action")

	def stop_scanning(self):
		rospy.loginfo("NTH - Stopping and centering laser")
		self.laser_center_pub.publish(True)
		if self.goal.get_goal().action==ScanningGoal.STOP_SCANNING:
			goal.set_succeeded(ScanningResult(ScanningResult.SUCCESS),
					"Success in stopping laser")
		else:
			goal.set_succeeded(ScanningResult(ScanningResult.WARNING),
					"Scan aborted")
		self.scanning_state = ScanningFeedback.READY
		goal.publish_feedback(ScanningFeedback(self.scanning_state))

	## forwarding scanning_once topic to a new goal
	def scanning_once_cb(self, speed):

		goal = ScanningGoal()
		goal.action = ScanningGoal.START_SCANNING
		goal.speed = speed.data
		self.action_client.send_goal(goal)

	def start_scanning(self, speed):
		# clip speed
		speed = max(min(speed, 0.1), 1.2)
		# send command
		rospy.loginfo("NTH - Starting scan")
		rospy.logdebug("NTH - Sending scanning speed command: %f and disabling \
publication of the messy point cloud."%speed)
		self.ptcld_ctrl_pub.publish(False)
		self.scanning_speed_pub.publish(speed)
		self.last_goal_time = get_time()
		# update state
		self.scanning_state = ScanningFeedback.WAITING_FOR_FIRST_SWIPE
		self.goal.publish_feedback(ScanningFeedback(self.scanning_state))
	


def main():
	try:
		# starts the node
		rospy.init_node('nth')
		# instantiate the class
		scan3d = ScanningService()
		fp = FlipperPosture()
		# wait until closed
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
	

