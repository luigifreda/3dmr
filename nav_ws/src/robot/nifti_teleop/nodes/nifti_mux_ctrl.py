#!/usr/bin/env python
import roslib; roslib.load_manifest('nifti_teleop')
import rospy
import rospy.rostime as rostime
from topic_tools.srv import MuxSelect
from nifti_teleop.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from threading import Lock
from time import sleep, time

## priority definition
# higher values have higher priority
priority = {
	"__none": -1,
	"/nav/cmd_vel": 0,
	"/teleop_ocu/cmd_vel": 2,
	"/teleop_joy/cmd_vel": 4,
	"/local_joy/cmd_vel": 10
}



## Mux priority control class
class NiftiMuxCtrl(object):
	def __init__(self):
		# getting access to the mux
		rospy.loginfo('Waiting for mux service...')
		rospy.wait_for_service('private/mux_cmd_vel/select')
		rospy.loginfo('Reached mux service.')
		self.mux_select = rospy.ServiceProxy('private/mux_cmd_vel/select', MuxSelect)

		self.lock = Lock()
		# setting up own services
		self.acquire_srv = rospy.Service('mux_cmd_vel/acquire', Acquire,
				self.handle_acquire)
		self.release_srv = rospy.Service('mux_cmd_vel/release', Release,
				self.handle_release)
		# setting initial state
		self.selected = None
		self.requesting = None
		self.stack = ['__none']
		self.mux_select('__none')
		# monitor actual mux state
		rospy.Subscriber('mux_cmd_vel/selected', String, self.selected_cb)

		# last messages
		topics = priority.keys()
		topics.remove('__none')
		self.last_msg = dict(zip(topics, [rostime.Time(0)]*len(topics)))
		for topic in topics:
			rospy.Subscriber(topic, Twist, self.cmd_vel_cb_factory(topic))


	def cmd_vel_cb_factory(self, topic):
		def cb(msg):
			if topic in self.stack:
				self.last_msg[topic] = rostime.get_rostime()
		return cb


	def handle_acquire(self, req):
		resp = False
		self.lock.acquire()
		while self.requesting:
			sleep(0.010)
		if priority[self.stack[-1]]<priority.get(req.topic, -1):
			rospy.logdebug('Giving lock to "%s".', req.topic)
			self.requesting = req.topic
			self.mux_select(req.topic)
			if not self.selected == req.topic:
				start = time()
				sleep(0.001)
				while (not self.selected == req.topic) and (time()-start<0.5):
					sleep(0.010)
			if self.selected == req.topic:
				resp = True
				rospy.logdebug("Lock given.")
			else:
				rospy.logwarn('Lock failed somehow for "%s".', req.topic)
		self.lock.release()
		return AcquireResponse(resp)


	def handle_release(self, req):
		resp = False
		self.lock.acquire()
		while self.requesting:
			sleep(0.010)
		if (req.topic in self.stack) and (req.topic != '__none'):
			rospy.logdebug('Releasing request for "%s".', req.topic)
			if self.stack[-1] == req.topic:
				self.stack.pop()
				self.requesting = self.stack[-1]
				self.mux_select(self.requesting)
			else:
				self.stack.remove(req.topic)
			self.last_msg[req.topic] = rostime.Time(0)
			rospy.logdebug("Request released.")
			resp = True
		self.lock.release()
		return ReleaseResponse(resp)
			
	def selected_cb(self, msg):
		rospy.loginfo('Selected: "%s"', msg.data)
		self.selected = msg.data
		if self.requesting and self.selected!=self.requesting:
			rospy.logerr('Requesting "%s" but selected is "%s"!',
					self.requesting, self.selected)
		self.requesting = None
		if self.stack[-1]!=self.selected:
			self.stack.append(self.selected)
		if msg.data != '__none':
			self.last_msg[msg.data] = rostime.get_rostime()

	def run(self):
		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			try:
				r.sleep()
			except rospy.exceptions.ROSTimeMovedBackwardsException, e:
				self.lock.acquire()
				rospy.logdebug(str(e))
				self.stack = ['__none']
				self.selected = None
				self.requesting = None
				self.mux_select('__none')
				self.lock.release()
				continue
			now = rostime.get_rostime()
			for topic, last in self.last_msg.items():
				if (not last.is_zero()) and ((now-last).to_sec()>2):
					# auto release lock
					self.lock.acquire()
					while self.requesting:
						sleep(0.010)
					rospy.logwarn('Timeout release of request for "%s".', topic)
					if self.stack[-1] == topic:
						self.stack.pop()
						self.requesting = self.stack[-1]
						self.mux_select(self.requesting)
					else:
						self.stack.remove(topic)
					self.last_msg[topic] = rostime.Time(0)
					self.lock.release()


## Create a ROS node and instantiate the class.
def main():
	'''Create a ROS node and instantiate the class.'''
	try:
		rospy.init_node('nifti_mux_control')
		nmc = NiftiMuxCtrl()
		nmc.run()
	except rospy.ROSInterruptException:
		pass


if __name__== '__main__':
	main()
	
	
