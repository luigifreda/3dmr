#!/usr/bin/env python

import copy
from operator import itemgetter
import math

import rospy
from geometry_msgs.msg import Twist, TwistStamped


class SpeedLimiter:

    def __init__(self):
        self._limit_listener = rospy.Subscriber("adapt_trav_vel_in", TwistStamped, self._on_limit, queue_size=100)
        self._limit_publisher = rospy.Publisher("adapt_trav_vel", TwistStamped, queue_size=100, latch=True)

        self._lin_limits = dict()
        self._ang_limits = dict()

        self._limit_msg = TwistStamped()

        self._cmd_vel_in_subscriber = rospy.Subscriber("cmd_vel_in", Twist, self._on_cmd_vel, queue_size=10)
        self._cmd_vel_out_publisher = rospy.Publisher("cmd_vel_out", Twist, queue_size=10)

        self._cmd_vel_msg = Twist()

        self._override_active = False

    def run(self):
        rospy.spin()

    def _on_limit(self, limit_message):
        assert isinstance(limit_message, TwistStamped)

        name = limit_message.header.frame_id
        if len(name) == 0:
            publisher_node_name = limit_message._connection_header['callerid']
            # rostopic nodes are anonymous, but we want to group all of them to one node
            if publisher_node_name.startswith("/rostopic"):
                publisher_node_name = "/rostopic"
            rospy.logerr("Publisher %s hasn't set the header.frame_id to identify itself!" % publisher_node_name)
            name = publisher_node_name

        speed_limit_lin = limit_message.twist.linear.x
        if speed_limit_lin < 0:
            rospy.logwarn("Speed limit has to be non-negative, %f given." % speed_limit_lin)
            return

        speed_limit_ang = limit_message.twist.angular.z
        if speed_limit_ang < 0:
            rospy.logwarn("Speed limit has to be non-negative, %f given." % speed_limit_ang)
            return

        rospy.logdebug("Received speed limit (%f, %f) from %s" % (
            speed_limit_lin, speed_limit_ang, name))

        if name == "override":
            if speed_limit_lin is not None and not math.isnan(speed_limit_lin):
                self._override_active = True
            else:
                self._override_active = False
            # we don't want further processing when override received
            return

        if speed_limit_lin is not None and not math.isnan(speed_limit_lin):
            self._lin_limits[name] = speed_limit_lin
        elif name in self._lin_limits.keys():
            del self._lin_limits[name]

        if speed_limit_ang is not None and not math.isnan(speed_limit_ang):
            self._ang_limits[name] = speed_limit_ang
        elif name in self._ang_limits.keys():
            del self._ang_limits[name]

        self.publish_current_limit()

    def publish_current_limit(self):

        ((current_lin_limit, current_lin_limit_source), (current_ang_limit, current_ang_limit_source)) = \
            self.get_current_limit()

        self._limit_msg.twist.linear.x = (current_lin_limit if current_lin_limit is not None else float('nan'))
        self._limit_msg.twist.angular.z = (current_ang_limit if current_ang_limit is not None else float('nan'))
        self._limit_msg.header.stamp = rospy.Time.now()
        self._limit_msg.header.frame_id = (current_lin_limit_source if current_lin_limit is not None else "") + ";" + \
                                          (current_ang_limit_source if current_ang_limit is not None else "")

        self._limit_publisher.publish(self._limit_msg)

    def get_current_limit(self):
        """Return the currently applicable speed limit.

        :return: tuple ((lin_limit, lin_limit_source), (ang_limit, ang_limit_source))
        """
        lin_limit = (None, None)
        if len(self._lin_limits) > 0:
            lin_limit = self._min_argmin(self._lin_limits)

        ang_limit = (None, None)
        if len(self._ang_limits) > 0:
            ang_limit = self._min_argmin(self._ang_limits)

        return lin_limit, ang_limit

    def _on_cmd_vel(self, cmd_vel):
        assert isinstance(cmd_vel, Twist)
        out_cmd_vel = copy.deepcopy(cmd_vel)

        if not self._override_active:

            ((lin_limit, _), (ang_limit, _)) = self.get_current_limit()

            if lin_limit is not None:
                if out_cmd_vel.linear.x >= 0 and out_cmd_vel.linear.x > lin_limit:
                    rospy.logdebug("Speed limiting decreased linear speed from %f to %f" % (out_cmd_vel.linear.x, lin_limit))
                    out_cmd_vel.linear.x = lin_limit
                elif out_cmd_vel.linear.x < 0 and out_cmd_vel.linear.x < -lin_limit:
                    rospy.logdebug("Speed limiting decreased linear speed from %f to %f" % (out_cmd_vel.linear.x, -lin_limit))
                    out_cmd_vel.linear.x = -lin_limit

            if ang_limit is not None:
                if out_cmd_vel.angular.z >= 0 and out_cmd_vel.angular.z > ang_limit:
                    rospy.logdebug("Speed limiting decreased angular speed from %f to %f" % (out_cmd_vel.angular.z, ang_limit))
                    out_cmd_vel.angular.z = ang_limit
                elif out_cmd_vel.angular.z < 0 and out_cmd_vel.angular.z < -ang_limit:
                    rospy.logdebug("Speed limiting decreased angular speed from %f to %f" % (out_cmd_vel.angular.z, -ang_limit))
                    out_cmd_vel.angular.z = -ang_limit

        self._cmd_vel_out_publisher.publish(out_cmd_vel)

    @staticmethod
    def _min_argmin(items):
        """Compute the argmin and min of a dictionary.

        :param dict items: The dictionary to search through.
        :return: tuple (min, argmin)
        """
        return tuple(reversed(min(items.iteritems(), key=itemgetter(1))))

if __name__ == '__main__':
    rospy.init_node('speed_limiter')
    limiter = SpeedLimiter()
    limiter.run()
