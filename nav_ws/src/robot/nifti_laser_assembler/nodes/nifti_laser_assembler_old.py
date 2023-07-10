#!/usr/bin/env python
## @file
# Laser assembler module for the NIFTi robot
#
# It follows the motion of the laser sensor to trigger the assembly of laser
# scans into a single point cloud.

import roslib; roslib.load_manifest('nifti_laser_assembler')
import rospy

import math
from math import pi, atan2
from tf.msg import tfMessage
from geometry_msgs.msg import *
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud


## Get the angle of the laser from the tf message
def laser_angle_from_tf(tf_message):
	theta_raw = 2*atan2(tf_message.transform.rotation.x,\
			tf_message.transform.rotation.w) - pi
	# put it back between -pi and pi
	if theta_raw>pi:
		theta = theta_raw-2*pi
	elif theta_raw<-pi:
		theta = theta_raw+2*pi
	else:
		theta = theta_raw
	return theta


## time interpolation
def interpolate(v1, t1, v2, t2):
	return (v2*t1 - v1*t2)/(v2-v1)


## Main class to assemble individual laser scans into point clouds
class NiftiLaserAssembler(object):

	## Instantiate NiftiLaserAssembler object
	def __init__(self):
		#self.tf_listener = tf.TransformListener()
		# loop control
		#self.loop_rate = rospy.Rate(20.0)
		rospy.wait_for_service("assemble_scans")
		## service to assemble scans into PointCloud
		self.assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
		## publisher for the resulting point clouds
		self.pointCloud_pub = rospy.Publisher('nifti_point_cloud', PointCloud)
		## subscriber for tf
		rospy.Subscriber('/tf', tfMessage, self.tf_cb)
		## previous laser angle value
		self.old_abs_angle = None
		## previous time stamp
		self.old_time = None
		## starting time for current swipe
		self.start_time = None

	## callback for the tf messages
	def tf_cb(self, tf_message):
		for tmp_tf in tf_message.transforms:
			if (tmp_tf.header.frame_id == '/base_link') and\
					(tmp_tf.child_frame_id == '/laser'):
				abs_angle = abs(laser_angle_from_tf(tmp_tf))
				time = tmp_tf.header.stamp.to_sec()
	
				if (abs_angle<=pi/2) and (self.old_abs_angle>pi/2):
					self.start_time = interpolate(self.old_abs_angle-pi/2,
							self.old_time, abs_angle-pi/2, time)
					rospy.loginfo("Interpolated start time: %f"%self.start_time)

				if (abs_angle>=pi/2) and (self.old_abs_angle<pi/2):
					stop_time = interpolate(self.old_abs_angle-pi/2,
							self.old_time, abs_angle-pi/2, time)
					rospy.loginfo("Interpolated stop time: %f"%stop_time)
					self.assemble_and_publish(self.start_time, stop_time)

				self.old_abs_angle = abs_angle
				self.old_time = time


	## function that actually requests and sends the point cloud
	def assemble_and_publish(self, start, stop):
		if start is None:
			return
		rospy.loginfo("requesting assembly")
		resp = self.assemble_scans(rospy.Duration.from_sec(start),
				rospy.Duration.from_sec(stop))
		rospy.loginfo("publishing cloud")
		self.pointCloud_pub.publish(resp.cloud)
		rospy.logwarn('Debug - got %u points' % len(resp.cloud.points))
		rospy.loginfo("done")
	


	def spin(self):
		theta_old = 0.0
		start_time = None
		while not rospy.is_shutdown():
			try:
				trans, rot = self.tf_listener.lookupTransform('/laser',
					'/base_link', rospy.Time(0))

			except (tf.LookupException, tf.ConnectivityException):
				continue
			#print trans, rot
			# get angle from tf
			theta_raw = 2*atan2(rot[0], rot[3]) - pi
			# put it back between -pi and pi
			if theta_raw>pi:
				theta = theta_raw-2*pi
			elif theta_raw<-pi:
				theta = theta_raw+2*pi
			else:
				theta = theta_raw
			#print theta_raw, theta

			# get start time
			if ((theta <= pi/2) and (theta_old > pi/2)) or \
					((theta >= -pi/2) and (theta_old < -pi/2)):
				start_time = rospy.get_rostime()
				rospy.loginfo("start time for assembler")
			# get stop time and publish point cloud
			if start_time and (((theta >= pi/2) and (theta_old < pi/2)) or \
					((theta <= -pi/2) and (theta_old > -pi/2))):
				rospy.loginfo("stop time: requesting assembly")
				resp = self.assemble_scans(start_time, rospy.get_rostime())
				rospy.loginfo("publishing cloud")
				self.pointCloud_pub.publish(resp.cloud)
				rospy.loginfo("done")

			theta_old = theta
			self.loop_rate.sleep()


## Create a ROS node and instantiate the NiftiLaserAssembler class.
def main():
	try:
		rospy.init_node('nifti_laser_assembler')
		nla = NiftiLaserAssembler()
		rospy.spin()
		#nla.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()
