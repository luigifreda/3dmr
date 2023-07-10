#!/usr/bin/env python

import os
import re
from xml import dom

import rospy
import rospkg
from dynamic_reconfigure.server import Server

import xacro

from nifti_robot_description.cfg import RobotPartsConfig


class DynamicRobotModelServer(object):
    """
    Dynamic reconfigure server handling the robot model generation.

    Before running this server, load the robot.yaml params to the ROS param server.
    This script then reads all `parts_config/*` entries on the param server and uses them to configure the model from
    urdf/nifti_robot.xacro (their names should correspond to names in the <arg> tags in the xacro file).

    All parts of the TRADR system for which it makes sense to somehow alter their state whenever a change to the robot
    body is done should not directly read the /nifti_robot_description and /parts_config/* values, but should instead
    create a dynamic reconfigure client and receive updates to the parameters on the fly.

    IMPLEMENTATION NOTE:
    This node uses a hack to dynamic_reconfigure to allow not only for dynamic values, but also for dynamic list of
    parameters. The implementation should work flawlessly in Python, but it seems that C++ support would be a bit
    more difficult and it has not been done yet. The core part of the hack is in method _augment_config_type.
    """

    def __init__(self):
        """
        Set up the dynamic model server, read params from param server, and generate an initial robot model.
        """
        super(DynamicRobotModelServer, self).__init__()

        self.__float_range = (-10.0, 10.0)
        self.__int_range = (-100, 100)

        self._xacro_file = os.path.join(rospkg.RosPack().get_path('nifti_robot_description'), 'urdf', 'nifti_robot.xacro')

        self._augment_config_type(RobotPartsConfig)
        self._server = Server(RobotPartsConfig, self.change_callback)

        self._last_config = self._server.config

    def change_callback(self, config, level):
        """
        Called whenever the config is changed externally.

        :param dynamic_reconfigure.Config config: The new config
        :param int level: Not used
        :return: An updated config (value errors corrected etc.)
        :rtype: dynamic_reconfigure.Config
        """

        try:
            robot_description = self._get_configured_urdf(config)

            config.robot_description = robot_description
            rospy.set_param('nifti_robot_description', robot_description)

            rospy.loginfo("Robot model updated, it has %i bytes." % len(robot_description))

            for key in self._get_parts_config().keys():
                rospy.set_param("parts_config/%s" % key, getattr(config, key))

            self._last_config = config

        except Exception, e:
            rospy.logerr('XACRO to URDF conversion did not succeed: %s, %r' % (str(e), e))
            config = self._last_config

        return config

    def _get_configured_urdf(self, config):
        """
        Call the XACRO to URDF conversion using the parameters given in `config`.

        :param dynamic_reconfigure.Config config: Arguments of the XACRO model.
        :return: The URDF file contents.
        :rtype: basestring
        """

        doc = xacro.parse(None, self._xacro_file)
        self.__remove_comments_from_xml_doc(doc)

        xacro.process_doc(doc, mappings=dict([(key, str(val)) for (key, val) in config.iteritems()]), in_order=True)

        return doc.toxml()

    @staticmethod
    def __remove_comments_from_xml_doc(doc):
        """
        Remove all XML comments from the given parsed document.

        :param xml.doc.minidom.Document doc: The document to remove comments from.
        """
        for tag in doc.getElementsByTagName("*"):
            for n in tag.childNodes:
                if n.nodeType is dom.Node.COMMENT_NODE:
                    n.parentNode.removeChild(n)

    def _augment_config_type(self, config_type):
        """
        Main part of the hack that allows us to have a dynamic list of dynamic parameters.

        It simulates what the compiled `*Config.py` files do, but the list of parameters is not statically written, it is
        instead built from all parameters found in the `parts_config/*` section on the parameter server.

        :param type config_type: The `*Config` base type which will be extended.
        """
        parts_config = self._get_parts_config()

        config_type.all_level = 0

        parts_keys = list(parts_config.keys())
        parts_keys.sort()

        for key in parts_keys:
            value = parts_config[key]

            value_type = type(value)

            # a corner case which is often present in our robot.yaml configs
            if key.startswith("has_") and (value_type == float or value_type == int):
                value_type = bool
                value = True if value != 0.0 else False
                rospy.logwarn("You should update your robot.yaml file so that '%s' has a 'True' or 'False' as value, "
                              "and not integers/floats." % key)

            description = {
                'description': key,
                'name': key,
                'edit_method': '',
                'level': 0,
                'type': value_type.__name__,
                'default': value,
            }

            config_type.level[key] = 0
            config_type.type[key] = value_type.__name__
            config_type.defaults[key] = value

            if value_type == bool:
                config_type.min[key] = False
                description['min'] = False

                config_type.max[key] = True
                description['max'] = True

            elif value_type == float:
                config_type.min[key] = self.__float_range[0]
                description['min'] = self.__float_range[0]

                config_type.max[key] = self.__float_range[1]
                description['max'] = self.__float_range[1]

                config_type.type[key] = 'double'
                description['type'] = 'double'

            elif value_type == int:
                config_type.min[key] = self.__int_range[0]
                description['min'] = self.__int_range[0]

                config_type.max[key] = self.__int_range[1]
                description['max'] = self.__int_range[1]

            elif value_type == str:
                config_type.min[key] = ''
                description['min'] = ''

                config_type.max[key] = ''
                description['max'] = ''

            config_type.config_description['parameters'].append(description)

    @staticmethod
    def _get_parts_config():
        """
        Read all `parts_config/*` entries from the parameter server and return a dict, where keys are the part names
        (without the "namespace" prefix).

        :return: All parts configuration.
        :rtype: dict
        """
        parts_config_matcher = re.compile('^.*parts_config/')
        parts_config = {}
        for param in rospy.get_param_names():
            if parts_config_matcher.match(param):
                value = rospy.get_param(param)
                param_name = parts_config_matcher.sub('', param)
                parts_config[param_name] = value

        return parts_config


if __name__ == '__main__':
    rospy.init_node('dynamic_robot_model_server')
    server = DynamicRobotModelServer()
    rospy.spin()
