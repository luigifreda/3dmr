#!/usr/bin/env python
## @file
# Set of helper tools for the teleoperation of the NIFTi robot
#
# Right now only ScanningService is implemented: it listens on a topic and
# handles the motion of the rolling laser.
import roslib; roslib.load_manifest('nifti_laser_assembler')
import rospy
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import PointCloud2
from nifti_robot_driver_msgs.msg import RobotStatus


## ScanningService class
class ScanningService(object):
	'''Handle the motion of the rolling laser for taking a single 3D scan.
	'''

	## Instantiate the ScanningService class
	def __init__(self):
		## scanning speed command publisher
		self.scanning_speed_pub = rospy.Publisher('/scanning_speed_cmd', Float64)
		## laser center command publisher
		self.laser_center_pub = rospy.Publisher('/laser_center', Bool)
		## robot status subscriber (to get the current speed of the laser)
		rospy.Subscriber('/robot_status', RobotStatus, self.status_cb)
		## scanning_once subscriber
		rospy.Subscriber('/scanning_once', Float64, self.scanning_once_cb)
		## end of swipe subscriber (to know when to stop)
		rospy.Subscriber('/end_of_swipe', Bool, self.end_of_swipe_cb)
		## point cloud control publisher
		self.ptcld_ctrl_pub = rospy.Publisher('/pointcloud_control', Bool)
		## scanning state ("Not scanning", "Starting scanning", "Got first cloud")
		self.scanning_state = "Not scanning"
		self.last_goal_time = get_time()

	
	## robot status callback to get the current speed of the laser
	def status_cb(self, robot_status):
		self.scanning_speed = robot_status.scanning_speed
# disabled: too much trouble
#		if (self.scanning_speed == 0.0) and \
#				(get_time() - self.last_goal_time>0.1):	
#			# if not moving, updating state
#			rospy.loginfo('NTH - Laser speed 0: setting state to "Not scanning".')
#			self.scanning_state = "Not scanning"


	## scanning once callback to start the laser
	def scanning_once_cb(self, speed):
		if self.scanning_state!="Not scanning": # don't do anything if we're already scanning
			return
		if self.scanning_speed: # don't do anything if the laser is already moving
			return
		# clip speed
		speed = max(min(speed, 0.1), 1.2)
		# send command
		rospy.logdebug("NLAH - Sending scanning speed command: %f and disabling \
publication of the messy point cloud."%speed)
		self.ptcld_ctrl_pub.publish(False)
		self.scanning_speed_pub.publish(speed)
		self.last_goal_time = get_time()
		# update state
		self.scanning_state = "Starting scanning"
	

	## point cloud callback to get when to stop the laser
	def end_of_swipe_cb(self, _):
		if self.scanning_state == "Starting scanning": # no end of swipe received yet
			rospy.logdebug("NLAH - Received first end of swipe: waiting for a \
second one and activating point cloud publication.")
			self.ptcld_ctrl_pub.publish(True)
			self.scanning_state = "Got first cloud"
			return
		elif self.scanning_state == "Got first cloud": #  first end of swipe received
			rospy.logdebug("NLAH - Received second end of swipe: stopping laser and \
centering it.")
			self.scanning_state = "Not scanning"
			self.scanning_speed_pub.publish(0.0) # may be unnecessary
			self.laser_center_pub.publish(True)
			return
		else:
			rospy.logdebug("NLAH - end of swipe received and ignored.")
			return

def main():
	try:
		# starts the node
		rospy.init_node('nifti_laser_assembler_helper')
		# instantiate the class
		scan3d = ScanningService()
		# wait until closed
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
	

