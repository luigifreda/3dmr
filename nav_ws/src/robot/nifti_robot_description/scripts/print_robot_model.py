#!/usr/bin/env python
# coding=utf-8

"""
Print the robot model based on nifti_robot.xacro, configured by the parts_config/* parameters on the param server or
from robot.yaml if the parameter server has not been running yet (or if the first or second arg is "yaml").

Usage:

    rosrun nifti_robot_description print_robot_model.py [xacro_file_path.xacro] [yaml|yaml_file_path.yaml] [[part:=value ]...]

If `xacro_file_path.xacro` is given, it is used as the input file for XACRO preprocessor instead of the default file
nifti_robot_description/urdf/nifti_robot.xacro . The filename extension MUST be .xacro .

If `yaml` (literally) is given, the config will be loaded from robot.yaml regardless of whether the parameter server is
running or not.
If `yaml_file_path.yaml` is given, it is used as the robot config instead of the default robot.yaml file.
The filename extension MUST be .yaml .

All other arguments are taken as parameter value specifications and override those read from the parameters or yaml.
"""

import os
import re
import socket
import sys
import yaml

import rospkg
import rospy
from rosgraph.names import load_mappings
import xacro

if __name__ == '__main__':

    args = sys.argv[1:]

    robot_xacro = None
    if len(args) > 0 and args[0].rstrip().endswith(".xacro"):
        path = args[0].strip()
        if os.path.exists(path) and os.path.isfile(path):
            robot_xacro = path
            args = args[1:]
        else:
            sys.stderr.write("The provided robot XACRO file %s doesn't exist.\n" % path)
            sys.exit(6)

    ros_params = None
    try:
        ros_params = rospy.get_param_names()
    except socket.error, e:
        # parameter server (roscore) not running, so read the config from robot.yaml
        pass

    robot_yaml_file = None
    if len(args) > 0:
        if args[0] == "yaml":
            ros_params = None
        elif args[0].rstrip().endswith(".yaml"):
            path = args[0].strip()
            if os.path.exists(path) and os.path.isfile(path):
                ros_params = None
                robot_yaml_file = path
            else:
                sys.stderr.write("The provided YAML config %s doesn't exist.\n" % path)
                sys.exit(5)
        args = args[1:]

    # read all parts_config/ parameters from the parameter server (they're initially loaded from robot.yaml)
    parts_config = {}
    namespace = rospy.get_namespace()
    if ros_params is not None:
        # ROS parameter server is running, so read the config from the parameters
        parts_config_matcher = re.compile('^' + namespace + '.*parts_config/')
        for param in ros_params:
            if parts_config_matcher.match(param):
                value = rospy.get_param(param)
                param_name = parts_config_matcher.sub('', param)  # names of the params correspond to <arg> tags in the Xacro
                parts_config[param_name] = value
        sys.stderr.write("Robot parts config taken from parameter server with the following values: %r\n" % parts_config)
    else:
        # read the config from robot.yaml
        try:
            nifti_launchers_path = rospkg.RosPack().get_path('nifti_launchers')
        except rospkg.ResourceNotFound, e:
            sys.stderr.write("Rospack is not working. Have you sourced the ROS environment in this shell? Error: %s\n"
                             % str(e))
            sys.exit(2)

        # load default-robot.yaml
        try:
            with open(os.path.join(nifti_launchers_path, 'launch', 'default-robot.yaml'), 'r') as robot_yaml:
                robot_yaml_values = yaml.load(robot_yaml)
            if robot_yaml_values is None:
                sys.stderr.write("File nifti_launchers/launch/default-robot.yaml is empty.\n")
                sys.exit(4)
            parts_config = robot_yaml_values['parts_config']
        except IOError, e:
            sys.stderr.write("Could not find nifti_launchers/launch/default-robot.yaml. Error: %s\n" % str(e))
            sys.exit(3)

        # update with robot.yaml specializations
        try:
            if robot_yaml_file is None:
                # if no YAML file is provided, use robot.yaml
                robot_yaml_file = os.path.join(nifti_launchers_path, 'launch', 'robot.yaml')

            with open(robot_yaml_file, 'r') as robot_yaml:
                robot_yaml_values = yaml.load(robot_yaml)
            if robot_yaml_values is not None and 'parts_config' in robot_yaml_values.keys():
                parts_config.update(robot_yaml_values['parts_config'])
            
            sys.stderr.write("Robot parts config taken from %s with the following values: %r\n" % (
                robot_yaml_file, parts_config))

        except IOError, e:
            sys.stderr.write("Could not find %s. Have you created the file?\nError: %s\n" % (robot_yaml_file,  str(e)))
            sys.exit(3)

    try:
        if robot_xacro is None:
            # if no xacro file is provided, use the default one
            package_path = rospkg.RosPack().get_path('nifti_robot_description')
            robot_xacro = os.path.join(package_path, 'urdf', 'nifti_robot.xacro')

        if len(args) > 0:
            # if other arguments are provided, treat them as ROS key:=value stuff and give them precedence over what is
            # on the parameter server
            mappings = load_mappings(args)
            parts_config.update(mappings)

        try:
            doc = xacro.process_file(robot_xacro,
                                     mappings=dict([(key, str(val)) for (key, val) in parts_config.iteritems()]),
                                     in_order=True, xacro_ns=False
                                     )
            print doc.toprettyxml()
        except Exception, e:
            sys.stderr.write("Error converting XACRO to URDF: %s, %r\n" % (str(e), e))
            sys.exit(4)

    except rospkg.ResourceNotFound, e:
        sys.stderr.write("Rospack is not working. Have you sourced the ROS environment in this shell? Error: %s\n"
                         % str(e))
        sys.exit(1)
