#!/usr/bin/env python
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
import rospy.rostime as rostime
from time import sleep
from geometry_msgs.msg import Twist
from nifti_teleop.srv import Acquire, Release


class PriorityWrapper(object):

	def __init__(self):
		self.output_topic = rospy.get_param('~output_topic',
				'nav/cmd_vel')
		self.input_topic = rospy.get_param('~input_topic',
				'private/nav/cmd_vel')
		## publisher for the velocity command topic
		# @param ~output_topic (default: 'nav/cmd_vel')
		self.cmdvel_pub = rospy.Publisher(self.output_topic, Twist)

		self.last_msg = rostime.Time(0)
		self.last_request = rostime.Time(0)

		# setting up priority requests
		gotit = False
		while not (gotit or rospy.is_shutdown()):
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
		## subscriber for the velocity command topic
		# @param ~input_topic (default: 'private/nav/cmd_vel')
		sub = rospy.Subscriber(self.input_topic, Twist, self.cmdvel_cb, queue_size=10)

	def cmdvel_cb(self, msg):
		now = rostime.get_rostime()
		#print(now), " ",
		if (now-self.last_request).to_sec()>0.5:
			self.priority_acquire(self.output_topic)
			self.last_request = now
		self.cmdvel_pub.publish(msg)
		self.last_msg = now
		#print(rostime.get_rostime())

	def run(self):
		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			r.sleep()
			if (not self.last_msg.is_zero()) and\
					(rostime.get_rostime()-self.last_msg).to_sec()>0.5:
				self.last_request = rostime.Time(0)
				self.last_msg = rostime.Time(0)
				self.priority_release(self.output_topic)


## Create a ROS node and instantiate the class.
def main():
	'''Create a ROS node and instantiate the class.'''
	try:
		rospy.init_node('priority_wrapper')
		pw = PriorityWrapper()
		pw.run()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
	

