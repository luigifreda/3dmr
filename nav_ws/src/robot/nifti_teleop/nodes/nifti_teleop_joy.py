#!/usr/bin/env python
## @file
# Joystick teleoperation module for the NIFTi robot
#
# It handles velocity command of the platform, activation of the brake, motion
# of the flippers, and selection of the rolling speed of the laser scanner.
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, Float64, Int32
from stamped_msgs.msg import Int32 as StampedInt32
# diamonback:
#from joy.msg import Joy
# electric and later:
from sensor_msgs.msg import Joy

from nifti_robot_driver_msgs.msg import FlippersState, RobotStatusStamped, FlipperCommand, FlippersStateStamped
from nifti_teleop.srv import Acquire, Release

from math import pi, floor, radians, isnan
from time import sleep

# TODO: should be part of the message definition
ID_FLIPPER_FRONT_LEFT=3
ID_FLIPPER_FRONT_RIGHT=4
ID_FLIPPER_REAR_LEFT=5
ID_FLIPPER_REAR_RIGHT=6

## Joystick teleoperation class
class NiftiTeleopJoy(object):
	'''Joystick teleoperation interface.

	It handles velocity command of the platform, activation of the brake, motion
	of the flippers, and selection of the rolling speed of the laser scanner.
	'''
	
	## Instantiate a NiftiTeleopJoy object
	def __init__(self):
		## current flippers state
		self.fs = FlippersState()
		self.fs.frontLeft = None 
		self.fs.frontRight = None
		self.fs.rearLeft = None
		self.fs.rearRight = None

		# joystick
		## proxy to enhance the Joy message with more functionalities (see
		# HistoryJoystick for details)
		self.joy = HistoryJoystick()
		
		# joystick axes
		## axis for linear velocity
		# @param ~axis_linear (default: 1)
		self.lin_vel_axis = rospy.get_param('~axis_linear', 1)
		## axis for angular velocity
		# @param ~axis_angular  (default: 0)
		self.ang_vel_axis = rospy.get_param('~axis_angular', 0)
		## axis for flippers command
		# @param ~flipper_axis (default: 5)
		self.flipper_axis = rospy.get_param('~flipper_axis', 5)
		## axis for scanning velocity
		# @param ~scanning_speed_axis (default: 4)
		self.scanning_speed_axis = rospy.get_param('~scanning_speed_axis', 4)
		## left-right axis of right thumbstick
		# @param ~right_ts_lr_axis  (default: 2)
		self.right_ts_lr_axis = rospy.get_param('~right_ts_lr_axis', 2)
		## up-down axis of right thumbstick
		# @param ~right_ts_ud_axis  (default: 3)
		self.right_ts_ud_axis = rospy.get_param('~right_ts_ud_axis', 3)
		# if no flipper control button is pressed, the up-down hat controls speed limit
		self.speed_limit_axis = self.flipper_axis
		
		# joystick buttons
		## deadman buttons
		# @param ~deadman_buttons (default: [1, 10])
		self.deadman_buttons = rospy.get_param('~deadman_buttons', [1,10])
		## run button
		# @param ~run_button (default: 0)
		self.run_button = rospy.get_param('~run_button', 0)
		## button for the enable command
		# @param ~enable_button (default: 9)
		self.enable_button = rospy.get_param('~enable_button', 9)
		## button to toggle the differential brake
		# @param ~differential_brake_button (default: 8)
		self.brake_button = rospy.get_param('~differential_brake_button', 8)
		## button to reset flippers to flat position
		# @param ~flipper_reset_button (default: 3)
		self.flipper_reset_button = rospy.get_param('~flipper_reset_button', 3)
		## button to select the front left flipper
		# @param ~front_left_flipper_button (default: 4)
		self.flipper_button_fl = rospy.get_param('~front_left_flipper_button', 4)
		## button to select the front right flipper
		# @param ~front_right_flipper_button (default: 5)
		self.flipper_button_fr = rospy.get_param('~front_right_flipper_button', 5)
		## button to select the rear left flipper
		# @param ~rear_left_flipper_button (default: 6)
		self.flipper_button_rl = rospy.get_param('~rear_left_flipper_button', 6)
		## button to select the rear right flipper
		# @param ~rear_right_flipper_button (default: 7)
		self.flipper_button_rr = rospy.get_param('~rear_right_flipper_button', 7)
		## scanning once button
		# @param ~scanning_once_button (default: 2)
		self.scanning_once_button = rospy.get_param('~scanning_once_button', 2)
		## mapping toggle button
		# @param ~mapping_toggle_button (default: 11)
		self.mapping_toggle_button = rospy.get_param('~mapping_toggle_button', 11)
		## button of right thumbstick
		# @param ~right_ts_button (default: 11)
		self.right_ts_button = rospy.get_param('~right_ts_button', 11)

		# speed limits
		## maximum linear velocity (in m/s)
		# @param /max_linear (default: 0.3)
		self.max_lin_vel = rospy.get_param('/max_linear', 0.3)
		## maximum angular velocity (in rad/s)
		# @param /max_angular (default: 0.6)
		self.max_ang_vel = rospy.get_param('/max_angular', 0.6)
		## maximum linear velocity with run button down (in m/s)
		# @param /max_linear_run (default: 0.6)
		self.max_lin_vel_run = rospy.get_param('/max_linear_run', 0.6)
		## maximum angular velocity with run button down (in rad/s)
		# @param /max_angular_run (default: 1.24)
		self.max_ang_vel_run = rospy.get_param('/max_angular_run', 1.24)
		## maximum scanning speed (in rad/s)
		# @param /max_scanning_speed (default: 1.20)
		self.max_scanning_speed = rospy.get_param('/max_scanning_speed', 1.20)
		## tracks distance (in m)
		# @param /tracks_distance (default: 0.397)
		self.tracks_distance = rospy.get_param('/tracks_distance', 0.397)
		## steering efficiency
		# @param /steering_efficiency (default: 0.41)
		self.steering_efficiency = rospy.get_param('/steering_efficiency', 0.41)
		## scanning speed increment when changing it (in rad/s)
		# @param ~scanning_speed_increment (default: 0.2)
		self.scanning_speed_increment = rospy.get_param('~scanning_speed_increment', 0.2)
		## flipper angle increment when moving them (in rad)
		# @param ~flipper_increment (default: 10*pi/180)
		self.flipper_increment = rospy.get_param('~flipper_increment', 20*pi/180.)
		## scanning once speed (in rad/s)
		# @param /scanning_once_speed (default: 1.20)
		self.scanning_once_speed = rospy.get_param('/scanning_once_speed', 1.20)

		## cmd_vel topic
		self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic',
				'teleop_joy/cmd_vel')

		## current mapping state
		self.mapping_on = True

		## last front command
		self.last_front_time = rospy.Time(0)
		## last rear command
		self.last_rear_time = rospy.Time(0)

		# the current linear speed maximum
		self.speed_limit = float('nan')
		# the current angular speed maximum
		self.speed_limit_ang = float('nan')

		# publishers
		## publisher for the velocity command topic
		# @param ~cmd_vel_topic (default: 'teleop_joy/cmd_vel')
		self.cmdvel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
		## publisher for the all flippers command topic
		self.flippers_pub = rospy.Publisher('flippers_cmd', FlippersState, queue_size=10)
		## publisher for the individual flipper command topic
		self.flipper_pub = rospy.Publisher('flipper_cmd', FlipperCommand, queue_size=10)
		## publisher for the brake command topic
		self.brake_pub = rospy.Publisher('brake', Bool, queue_size=1)
		## publisher for the enable command topic
		self.enable_pub = rospy.Publisher('enable', Bool, queue_size=1)
		## publisher for the scanning speed command topic
		self.scanning_speed_pub = rospy.Publisher('scanning_speed_cmd', Float64, queue_size=10)
		## publisher for the laser centering command topic
		self.laser_center_pub = rospy.Publisher('laser_center', Bool, queue_size=1)
		## publisher for the scanning once topic
		self.scanning_once_pub = rospy.Publisher('scanning_once', Float64, queue_size=1)
		## publisher for the mapping control topic
		#self.mapping_control_pub = rospy.Publisher('mapping_control', Bool, queue_size=1)
		## publisher for posture control
		self.posture_cmd_pub = rospy.Publisher('posture_cmd', Int32, queue_size=10)
		self.posture_cmd_stamped_pub = rospy.Publisher('posture_cmd_stamped', StampedInt32, queue_size=10)
		## publisher for a joy topic mixed from all connected joysticks
		self.any_joy_pub = rospy.Publisher('any_joy', Joy, queue_size=10)

		self.speed_limit_pub = rospy.Publisher('adapt_trav_vel_in', TwistStamped, queue_size=10)

		# setting up priority requests
		gotit = False
		while not gotit and not rospy.is_shutdown():
			try:
				rospy.wait_for_service('mux_cmd_vel/acquire', 10)
				rospy.wait_for_service('mux_cmd_vel/release', 10)
				gotit = True
			except rospy.ROSException:
				rospy.logwarn("Waiting for mux control services...")
			sleep(1)
		self.priority_acquire = rospy.ServiceProxy('mux_cmd_vel/acquire',
				Acquire)
		self.priority_release = rospy.ServiceProxy('mux_cmd_vel/release',
				Release)

		# subscribers
		## subscriber to the flippers state topic published by the robot driver
		# @param ~flippers_state_topic (default: '/flippers_state')
		rospy.Subscriber('flippers_state', FlippersStateStamped, self.flippersCallBack, queue_size=10)
		## subscriber to the steering_efficiency topic
		rospy.Subscriber('steering_efficiency', Float64, self.steering_efficiency_cb, queue_size=1)
		## subscriber to the robot status topic published by the robot driver
		rospy.Subscriber('robot_status', RobotStatusStamped, self.statusCallBack, queue_size=10)
		## subscriber to the joystick topic published by joy_node
		rospy.Subscriber('joy', Joy, self.joyCallBack, queue_size=10)



	## Update the steering efficiency
	def steering_efficiency_cb(self, msg):
		'''Update the steering efficiency.'''
		self.steering_efficiency = max(0.0, min(1.0, msg.data))

	## Listen to the status of the robot.
	def statusCallBack(self, robot_status):
		'''Listen to the status of the robot.'''
		## state of the brake (according to the driver)
		self.brake_on = robot_status.brake_on
		## scanning speed (according to the driver)
		self.scanning_speed = robot_status.scanning_speed

	## Listen to the flippers state.
	def flippersCallBack(self, flippers):
		'''Listen to the flippers state.'''
		# state of the flippers
		#self.flippers = flippers

		def roundFlipperValue(angle, inc=1.0*self.flipper_increment):
			return inc*round(angle/inc)
		self.fs.frontLeft = roundFlipperValue(flippers.frontLeft)
		self.fs.frontRight = roundFlipperValue(flippers.frontRight)
		self.fs.rearLeft = roundFlipperValue(flippers.rearLeft)
		self.fs.rearRight = roundFlipperValue(flippers.rearRight)


	## Callback for a joystick message.
	#
	# This method dispatches the message to each of the methods handling a
	# specific aspect:
	# - cmdvel_jcb,
	# - flipper_jcb,
	# - brake_jcb,
	# - enable_jcb,
	# - scanning_speed_jcb.
	def joyCallBack(self, joy):
		'''Callback for a joystick message.

		This method dispatches the message to each of the methods handling a
		specific aspect:
			- cmdvel_jcb,
			- flipper_jcb,
			- brake_jcb,
			- enable_jcb,
			- scanning_speed_jcb.'''
		self.joy.update(joy)
		self.cmdvel_jcb(self.joy)
		self.flipper_jcb(self.joy)
		self.brake_jcb(self.joy)
		self.enable_jcb(self.joy)
		self.scanning_speed_jcb(self.joy)
		self.scanning_once_jcb(self.joy)
		#self.mapping_control_jcb(self.joy)
		self.easy_flipper_jcb(self.joy)
		self.speed_limit_jcb(self.joy)
		self.any_joy_pub.publish(joy)

		self.mux_jcb(self.joy)

	def easy_flipper_jcb(self, joy):
		fb = -joy.axes[self.right_ts_lr_axis]
		ud = joy.axes[self.right_ts_ud_axis]
		if joy.pressed(self.right_ts_button):
			fs = FlippersState()
			stamped_msg = StampedInt32()
			stamped_msg.header = joy.header
			if fb>0.75 and abs(ud)<0.25:
				self.posture_cmd_pub.publish(2)
				stamped_msg.data = 2
			if ud<-0.75 and abs(fb)<0.25:
				self.posture_cmd_pub.publish(4)
				stamped_msg.data = 4
			if ud>0.75 and abs(fb)<0.25:
				self.posture_cmd_pub.publish(1)
				stamped_msg.data = 1
			if fb<-0.75 and abs(ud)<0.25:
				self.posture_cmd_pub.publish(3)
				stamped_msg.data = 3
			if abs(fb)<0.25 and abs(ud)<0.25:
				self.posture_cmd_pub.publish(0)
				stamped_msg.data = 0
			self.posture_cmd_stamped_pub.publish(stamped_msg)
#		elif joy.is_down_any(self.deadman_buttons):
#			if ud<-0.5:
#				flipper_change = -self.flipper_increment
#			elif ud>0.5:
#				flipper_change = self.flipper_increment
#			else:
#				return
#			flipperMotion = FlipperCommand()
#			now = rospy.Time.now()
#			if fb>0.5:
#				if (now-self.last_front_time).to_sec()>0.5:
#					rospy.loginfo("Moving front")
#					self.last_front_time = now
#					flipperMotion.object_id = ID_FLIPPER_FRONT_LEFT
#					flipperMotion.angle = self.fs.frontLeft - flipper_change
#					self.flipper_pub.publish(flipperMotion)
#					flipperMotion.object_id = ID_FLIPPER_FRONT_RIGHT
#					flipperMotion.angle = self.fs.frontRight - flipper_change
#					self.flipper_pub.publish(flipperMotion)
#			elif fb<-0.5:
#				if (now-self.last_rear_time).to_sec()>0.5:
#					self.last_rear_time = now
#					flipperMotion.object_id = ID_FLIPPER_REAR_LEFT
#					flipperMotion.angle = self.fs.rearLeft + flipper_change
#					self.flipper_pub.publish(flipperMotion)
#					flipperMotion.object_id = ID_FLIPPER_REAR_RIGHT
#					flipperMotion.angle = self.fs.rearRight + flipper_change
#					self.flipper_pub.publish(flipperMotion)




	## Handle priority based on the joystick input.
	def mux_jcb(self, joy):
                # this is needed for proper function even when the node runs in a namespace
                topic = self.cmd_vel_topic if self.cmd_vel_topic[0] == '/' else '/' + self.cmd_vel_topic

		if joy.pressed_any(self.deadman_buttons):
			self.priority_acquire(topic)
		elif joy.released_all(self.deadman_buttons):
			self.priority_release(topic)



	## Handle velocity command based on the joystick input.
	def cmdvel_jcb(self, joy):
		'''Handle velocity command based on the joystick input.'''
		tw = Twist()
		if joy.is_down_any(self.deadman_buttons):
			if joy.buttons[self.run_button]:
				lin_scale = self.max_lin_vel_run
				ang_scale = self.max_ang_vel_run
			else:
				lin_scale = self.max_lin_vel
				ang_scale = self.max_ang_vel
			# limit tracks speed
			v = lin_scale*joy.axes[self.lin_vel_axis]
			w = ang_scale*joy.axes[self.ang_vel_axis]
			l = v - w*self.tracks_distance/2./self.steering_efficiency
			r = v + w*self.tracks_distance/2./self.steering_efficiency
			Z = 1.
			if abs(l)>self.max_lin_vel_run:
				Z = abs(l)/self.max_lin_vel_run
			if abs(r)>self.max_lin_vel_run:
				Z = abs(r)/self.max_lin_vel_run
			r = r / Z
			l = l / Z
			v = (l + r)/2.
			w = (r - l)*self.steering_efficiency/self.tracks_distance

			tw.linear.x = v
			tw.angular.z = w

			self.cmdvel_pub.publish(tw)
		elif joy.released_all(self.deadman_buttons):	# make sure we ask the robot to stop
			tw.linear.x = 0.0
			tw.angular.z = 0.0
			self.cmdvel_pub.publish(tw)
		#else:
		#	pass	# we don't publish constantly

	## Get the indices of the four buttons controlling individual flipper motion
	def get_flipper_buttons(self):
		return [self.flipper_button_fl,
				self.flipper_button_fr,
				self.flipper_button_rl,
				self.flipper_button_rr]

	## Handle flippers command based on the joystick input.
	def flipper_jcb(self, joy):
		'''Handle flippers command based on the joystick input.'''
		if joy.is_down_any(self.deadman_buttons):
			if joy.pressed(self.flipper_reset_button):	# flat button
				self.posture_cmd_pub.publish(0)
			elif joy.axis_touched(self.flipper_axis):
				try:
					if all([joy.buttons[button] for button in self.get_flipper_buttons()]):
						self.fs.frontLeft -= joy.axes[self.flipper_axis]*self.flipper_increment
						self.fs.frontRight -= joy.axes[self.flipper_axis]*self.flipper_increment
						self.fs.rearLeft += joy.axes[self.flipper_axis]*self.flipper_increment
						self.fs.rearRight += joy.axes[self.flipper_axis]*self.flipper_increment
						self.flippers_pub.publish(self.fs)
					else:	# individual control
						flipperMotion = FlipperCommand()
						if joy.buttons[self.flipper_button_fl]:
							flipperMotion.object_id = ID_FLIPPER_FRONT_LEFT
							flipperMotion.angle = self.fs.frontLeft - joy.axes[self.flipper_axis]*self.flipper_increment
							self.flipper_pub.publish(flipperMotion)
						if joy.buttons[self.flipper_button_fr]:
							flipperMotion.object_id = ID_FLIPPER_FRONT_RIGHT
							flipperMotion.angle = self.fs.frontRight - joy.axes[self.flipper_axis]*self.flipper_increment
							self.flipper_pub.publish(flipperMotion)
						if joy.buttons[self.flipper_button_rl]:
							flipperMotion.object_id = ID_FLIPPER_REAR_LEFT
							flipperMotion.angle = self.fs.rearLeft + joy.axes[self.flipper_axis]*self.flipper_increment
							self.flipper_pub.publish(flipperMotion)
						if joy.buttons[self.flipper_button_rr]:
							flipperMotion.object_id = ID_FLIPPER_REAR_RIGHT
							flipperMotion.angle = self.fs.rearRight + joy.axes[self.flipper_axis]*self.flipper_increment
							self.flipper_pub.publish(flipperMotion)
				except (TypeError, AttributeError), e:
					rospy.logwarn('Flipper command ignored since no FlippersState message received.')


	## Handle brake command based on the joystick input.
	def brake_jcb(self, joy):
		'''Handle brake command based on the joystick input.'''
		if joy.is_down_any(self.deadman_buttons) and joy.pressed(self.brake_button):
			try:
				self.brake_pub.publish(not self.brake_on)
			except AttributeError:
				rospy.logwarn('Brake command ignored since no RobotStatus message received.')


	## Handle scanning_once command based on the joystick input.
	def scanning_once_jcb(self, joy):
		'''Handle scanning_once command based on the joystick input.'''
		if joy.is_down_any(self.deadman_buttons) and joy.pressed(self.scanning_once_button):
			rospy.loginfo('Initiating 3D scan at speed %f rad/s.'\
					%self.scanning_once_speed)
			self.scanning_once_pub.publish(self.scanning_once_speed)


	## Handle scanning_once command based on the joystick input.
	def mapping_control_jcb(self, joy):
		'''Handle mapping_control command based on the joystick input.'''
		if joy.is_down_any(self.deadman_buttons) and joy.pressed(self.mapping_toggle_button):
			if self.mapping_on:
				rospy.loginfo('Disabling mapping.')
			else:
				rospy.loginfo('Enabling mapping.')
			self.mapping_control_pub.publish(not self.mapping_on)
			self.mapping_on = not self.mapping_on

	## Handle scanning_once command based on the joystick input.
	def speed_limit_jcb(self, joy):
		'''Handle speed limit based on the joystick input.'''

		# special override behavior triggered by pressing both dead man's buttons
		if joy.is_down_all(self.deadman_buttons):
			twist = TwistStamped()
			twist.header.stamp = rospy.Time.now()
			twist.header.frame_id = "override"
			twist.twist.linear.x = self.max_lin_vel_run
			twist.twist.angular.z = self.max_ang_vel_run

			self.speed_limit_pub.publish(twist)
			return

		# one of dead man's buttons was just released
		elif joy.was_down_all(self.deadman_buttons):
			twist = TwistStamped()
			twist.header.stamp = rospy.Time.now()
			twist.header.frame_id = "override"
			twist.twist.linear.x = float('nan')
			twist.twist.angular.z = float('nan')

			self.speed_limit_pub.publish(twist)
			return

		publish = False

		if joy.is_down_any(self.deadman_buttons) and \
			joy.axis_touched(self.speed_limit_axis) and \
			not joy.is_down_any(self.get_flipper_buttons()):

			publish = True

			if isnan(self.speed_limit):
				self.speed_limit = self.max_lin_vel_run

			if joy.axes[self.speed_limit_axis] >= 0:
				self.speed_limit += 0.1
			else:
				self.speed_limit -= 0.1
			self.speed_limit = min(self.max_lin_vel_run, max(0, self.speed_limit))

		if joy.is_down(self.run_button) and \
				joy.axis_touched(self.speed_limit_axis) and \
				not joy.is_down_any(self.get_flipper_buttons()):

			publish = True

			if isnan(self.speed_limit_ang):
				self.speed_limit_ang = self.max_ang_vel_run

			if joy.axes[self.speed_limit_axis] >= 0:
				self.speed_limit_ang += 0.1
			else:
				self.speed_limit_ang -= 0.1
			self.speed_limit_ang = min(self.max_ang_vel_run, max(0, self.speed_limit_ang))

		if publish:
			twist = TwistStamped()
			twist.header.stamp = rospy.Time.now()
			twist.header.frame_id = "local_joy" if self.cmd_vel_topic.startswith("local_joy") else "joy"
			twist.twist.linear.x = self.speed_limit
			twist.twist.angular.z = self.speed_limit_ang

			self.speed_limit_pub.publish(twist)

	## Handle enable command based on the joystick input.
	def enable_jcb(self, joy):
		'''Handle enable command based on the joystick input.'''
		if joy.is_down_any(self.deadman_buttons) and joy.pressed(self.enable_button):
			self.enable_pub.publish(True)


	## Handle scanning speed command based on the joystick input.
	def scanning_speed_jcb(self, joy):
		'''Handle scanning speed command based on the joystick input.'''
		if joy.is_down_any(self.deadman_buttons) and\
				joy.axis_touched(self.scanning_speed_axis):
			try:
				v = self.scanning_speed-self.scanning_speed_increment*joy.axes[self.scanning_speed_axis]
				if v<0.:
					v = 0.
					self.laser_center_pub.publish(True)
				else:
					v = min(self.max_scanning_speed, v)
				self.scanning_speed_pub.publish(v)
			except AttributeError:
				rospy.logwarn('Scanning speed change command ignored since no\
				RobotStatus message receive.')

	## Work to do when the node is shutting down
	def shutdown(self):

		# cancel the speed limit
		twist = TwistStamped()
		twist.header.stamp = rospy.Time.now()
		twist.header.frame_id = "local_joy" if self.cmd_vel_topic.startswith("local_joy") else "joy"
		twist.twist.linear.x = float('nan')
		twist.twist.angular.z = float('nan')

		self.speed_limit_pub.publish(twist)

################################################################################



## Class to allow detecting transition in buttons and axes.
#
# It can be used like a Joy message but with additional methods.
class HistoryJoystick(Joy):
	'''Class to allow detecting transition in buttons and axes.'''

	## Instantiate a HistoryJoystick object
	def __init__(self):
		## Current state of the buttons
		self.buttons = None
		## Current state of the axes
		self.axes = None
		## Header
		self.header = None

	## To be called with each new joystick data.
	def update(self, joy):
		'''To be called with each new joystick data.'''
		## Previous state of the buttons
		self.old_buttons = self.buttons
		## Previous state of the axes
		self.old_axes = self.axes
		self.buttons = joy.buttons
		self.axes = joy.axes
		self.header = joy.header

	## Check if a given button is currently pressed down (state 1)
	def is_down(self, button_id):
		'''Check if a given button is currently pressed down (state 1).'''
		return self.is_down_any((button_id,))

	## Check if any of given buttons is currently pressed down (state 1)
	def is_down_any(self, button_ids):
		'''Check if any of given button is currently pressed down (state 1).'''
		try:
			res = False
			for button_id in button_ids:
				res = res or self.buttons[button_id]
			return res
		except TypeError:
			# no joystick messages yet?
			return False

	## Check if all of given buttons are currently pressed down (state 1)
	def is_down_all(self, button_ids):
		'''Check if all of given button are currently pressed down (state 1).'''
		try:
			res = True
			for button_id in button_ids:
				res = res and self.buttons[button_id]
			return res
		except TypeError:
			# no joystick messages yet?
			return False

	## Check if a given button was pressed down (state 1)
	def was_down(self, button_id):
		'''Check if a given button was pressed down (state 1).'''
		return self.was_down_any((button_id,))

	## Check if any of given buttons was pressed down (state 1)
	def was_down_any(self, button_ids):
		'''Check if any of given button was pressed down (state 1).'''
		try:
			res = False
			for button_id in button_ids:
				res = res or self.old_buttons[button_id]
			return res
		except TypeError:
			# no joystick messages yet?
			return False

	## Check if all of given buttons were pressed down (state 1)
	def was_down_all(self, button_ids):
		'''Check if all of given button were pressed down (state 1).'''
		try:
			res = True
			for button_id in button_ids:
				res = res and self.old_buttons[button_id]
			return res
		except TypeError:
			# no joystick messages yet?
			return False

	## Check if a given button has just been pressed (transition 0->1).
	def pressed(self, button_id):
		'''Check if a given button has just been pressed (transition 0->1).'''
		return self.pressed_any((button_id,))

	## Check if at least one in a given list of buttons has just been pressed (transition 0->1).
	def pressed_any(self, button_ids):
		'''Check if at least one in a given list of buttons has just been pressed (transition 0->1).'''
		try:
			return self.is_down_any(button_ids) and\
					not self.was_down_any(button_ids)
		except (AttributeError, TypeError), e:
			# not enough joystick messages yet?
			return False

	## Check if a given button has just been released (transition 1->0).
	def released(self, button_id):
		'''Check if a given button has just been released (transition 1->0).'''
		return self.released_any((button_id,))

	## Check if any of given buttons has just been released (transition 1->0).
	def released_any(self, button_ids):
		'''Check if any of given buttons has just been released (transition 1->0).'''
		try:
			res = False
			for button_id in button_ids:
				res = res or (not self.buttons[button_id] and\
					self.old_buttons[button_id])
			return res
		except (AttributeError, TypeError), e:
			# not enough joystick messages yet?
			return False

	## Check if all of given buttons have just been released (transition 1->0).
	def released_all(self, button_ids):
		'''Check if all of given buttons have just been released (transition 1->0).'''
		try:
			return not self.is_down_any(button_ids) and\
					self.was_down_any(button_ids)
		except (AttributeError, TypeError), e:
			# not enough joystick messages yet?
			return False

	## Check if a given button state has just changed (either transitions).
	def button_changed(self, button_id):
		'''Check if a given button state has just changed (either transitions).'''
		return self.button_changed_any((button_id,))

	## Check if any of given buttons state has just changed (either transitions).
	def button_changed_any(self, button_ids):
		'''Check if any of given buttons state has just changed (either transitions).'''
		try:
			res = False
			for button_id in button_ids:
				res = res or (self.buttons[button_id] !=\
					self.old_buttons[button_id])
			return res
		except (TypeError, AttributeError), e:
			# not enough joystick messages yet?
			return False

	## Check if a given axis has moved.
	def axis_moved(self, axis_id):
		'''Check if a given axis has moved.'''
		try:
			return self.axes[axis_id] != \
					self.old_axes[axis_id]
		except (TypeError, AttributeError), e:
			# no joystick messages yet?
			return False

	## Check if a given axis that was released has moved (transition 0->anything
	# else).
	def axis_touched(self, axis_id):
		'''Check if a given axis that was released has moved (transition
		0->anything else).'''
		try:
			return self.axes[axis_id] and \
					not self.old_axes[axis_id]
		except (TypeError, AttributeError), e:
			# no joystick messages yet?
			return False

	## Check if a given axis has just been released (transition anything
	# else->0).
	def axis_released(self, axis_id):
		'''Check if a given axis has just been released has (transition anything
		else->0).'''
		try:
			return not self.axes[axis_id] and \
					self.old_axes[axis_id]
		except (TypeError, AttributeError), e:
			# no joystick messages yet?
			return False
	################################################################################



## Create a ROS node and instantiate the NiftiTeleopJoy class.
def main():
	'''Create a ROS node and instantiate the NiftiTeleopJoy class.'''
	rospy.init_node('nifti_teleop_joy')
	ntj = NiftiTeleopJoy()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	ntj.shutdown()


if __name__== '__main__':
	main()
	
	
