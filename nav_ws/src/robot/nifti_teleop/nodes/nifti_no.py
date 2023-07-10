#!/usr/bin/env python
## @file
# Joystick teleoperation module for the NIFTi robot
#
# It handles velocity command of the platform, activation of the brake, motion
# of the flippers, and selection of the rolling speed of the laser scanner.
import roslib; roslib.load_manifest('nifti_teleop')
import rospy, time
from std_msgs.msg import Bool, Float64

scanning_speed_pub = rospy.Publisher('/scanning_speed_cmd', Float64)
center_pub = rospy.Publisher('/laser_center', Bool)

rospy.init_node('nifti_no')
time.sleep(5)
print 'Ok'

scanning_speed_pub.publish(0.7)
time.sleep(0.8)
center_pub.publish(True)
time.sleep(1.1)
scanning_speed_pub.publish(0.7)
time.sleep(.8)
center_pub.publish(True)

	
