#!/usr/bin/env python
## @file
# Joystick teleoperation module for the NIFTi robot in tank mode

import roslib; roslib.load_manifest('nifti_teleop')
import rospy
from joy.msg import Joy		# diamondback and less
#from sensor_msgs.msg import Joy	 # electric and more

from nifti_robot_driver_msgs.msg import Tracks

## Joystick teleoperation class
class NiftiTeleopTank(object):
	'''Joystick teleoperation interface.
	'''
	
	## Instantiate a NiftiTeleopJoy object
	def __init__(self):
		## proxy to enhance the Joy message with more functionalities (see
		# HistoryJoystick for details)
		self.joy = HistoryJoystick()
		
		# publisher and subscribers
		## publisher for the tracks velocity command
		self.trackscmd_pub = rospy.Publisher('/tracks_vel_cmd', Tracks)

		## subscriber to the joystick topic published by joy_node
		rospy.Subscriber('/joy', Joy, self.joyCallBack)


	## Callback for a joystick message.
	def joyCallBack(self, joy):
		'''Callback for a joystick message.'''
		self.joy.update(joy)
		if self.joy.buttons[3]:
			self.trackscmd_pub.publish(0.3*self.joy.axes[1],
						0.4*self.joy.axes[1])
		if self.joy.axis_moved(1) or self.joy.axis_moved(3):
			if self.joy.buttons[2]:
				self.trackscmd_pub.publish(0.5*self.joy.axes[1],
						0.5*self.joy.axes[1])
			else:
				self.trackscmd_pub.publish(0.5*self.joy.axes[1],
						0.5*self.joy.axes[3])

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

	## To be called with each new joystick data.
	def update(self, joy):
		'''To be called with each new joystick data.'''
		## Previous state of the buttons
		self.old_buttons = self.buttons
		## Previous state of the axes
		self.old_axes = self.axes
		self.buttons = joy.buttons
		self.axes = joy.axes

	## Check if a given button is currently pressed down (state 1)
	def is_down(self, button_id):
		'''Check if a given button is currently pressed down (state 1).'''
		try:
			return self.buttons[button_id]
		except TypeError:
			# no joystick messages yet?
			return False

	## Check if a given button has just been pressed (transition 0->1).
	def pressed(self, button_id):
		'''Check if a given button has just been pressed (transition 0->1).'''
		try:
			return self.buttons[button_id] and\
					not self.old_buttons[button_id]
		except (AttributeError, TypeError), e:
			# not enough joystick messages yet?
			return False

	## Check if a given button has just been released (transition 1->0).
	def released(self, button_id):
		'''Check if a given button has just been released (transition 1->0).'''
		try:
			return not self.buttons[button_id] and\
					self.old_buttons[button_id]
		except (AttributeError, TypeError), e:
			# not enough joystick messages yet?
			return False

	## Check if a given button state has just changed (either transitions).
	def button_changed(self, button_id):
		'''Check if a given button state has just changed (either transitions).'''
		try:
			return self.buttons[button_id] != \
					self.old_buttons[button_id]
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



## Create a ROS node and instantiate the NiftiTeleopTank class.
def main():
	'''Create a ROS node and instantiate the NiftiTeleopTank class.'''
	try:
		rospy.init_node('nifti_teleop_tank')
		ntt = NiftiTeleopTank()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
	
	
