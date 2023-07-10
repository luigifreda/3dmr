#!/usr/bin/env python
import os
import re
from socket import gaierror
import traceback
import yaml

import rospy
from rospkg import RosPack
from sensor_msgs.msg import Joy


def find_config_for_device(device):
    '''Tries to find a YAML config file for the joystick translation.

    It first looks, if the device isn't specified as /dev/input/by-id/$ID, and if it is, the config would be $ID.yaml.
    Then it tries to match $ID against all config files with string SERIAL omitted from both the config file name 
    and the device name (so that it matches all devices starting with the same name, but having a different serial 
    number in the device name).

    Otherwise, it resolves any possible symlinks in the device file to get the /dev/input/js$X name of the device.
    It then uses js$X to find a $VENDOR:$PRODUCT pair in /proc/bus/input/devices for this device. If it finds this
    information, it returns the config file $VENDOR:$PRODUCT.yaml (if it exists).

    If the config file cannot be found, it returns None.
    '''

    # we can assume the device file exists

    base_path = os.path.join(RosPack().get_path("nifti_teleop"), "conf", "joy_mappings")
    abs_device = device

    # first try utilizing $ID
    if os.path.islink(device):
        if device.startswith("/dev/input/by-id/"):
            device_name = os.path.basename(device)
            conf_file = os.path.join(base_path, device_name + ".yaml")
            
            # if we have an exact match for the device name, return it
            if os.path.exists(conf_file):
                return conf_file
            
            # otherwise try, if we can't match against some config with the SERIAL variable
            for conf_file in os.listdir(base_path):
                if conf_file.endswith(".yaml") and conf_file.find("SERIAL") >= 0:
                    conf_file_no_ext = os.path.splitext(conf_file)[0]
                    parts = conf_file_no_ext.split("SERIAL")

                    if device_name.startswith(parts[0]) and device_name.endswith(parts[1]):
                        return os.path.join(base_path, conf_file)

        abs_device = os.path.realpath(device)

    if re.match(r'^js[0-9]+$', os.path.basename(abs_device)) is None:
        rospy.logerr("The provided device file %s is not a joystick device file." % device)
        return None

    js_name = os.path.basename(abs_device)

    # otherwise search /proc/bus/input/devices to get the $VENDOR:PRODUCT information
    devices_file = "/proc/bus/input/devices"
    if os.path.exists(devices_file):
        handlers_pattern = re.compile(r'^H: Handlers=.* ' + js_name + r'\s*$')
        vendor_product_pattern = re.compile(r'Vendor=([0-9a-fA-F]+) Product=([0-9a-fA-F]+) ')
        name_pattern = re.compile(r'Name="(.*)"\s*$')
        with open(devices_file, 'r') as devices:
            device_vendor_product = None
            device_name = None
            found = False
            for line in devices:
                if line == "\n":
                    device_vendor_product = None
                    device_name = None
                elif re.match(handlers_pattern, line):
                    found = True
                    break
                elif line.startswith("I:"):
                    match = re.search(vendor_product_pattern, line)
                    if match is not None:
                        device_vendor_product = (match.group(1).lower(), match.group(2).lower())
                elif line.startswith("N:"):
                    match = re.search(name_pattern, line)
                    if match is not None:
                        device_name = match.group(1)

            if not found or device_vendor_product is None:
                rospy.logerr("Couldn't find device information for device %s." % device)
                return None

            rospy.loginfo("Found device %s with Vendor:Product=%s:%s" % (
                device_name, device_vendor_product[0], device_vendor_product[1]))

            vendor_product_config_file = os.path.join(base_path, "%s:%s.yaml" % device_vendor_product)
            if os.path.exists(vendor_product_config_file):
                return vendor_product_config_file

            rospy.logerr("Couldn't find config file by vendor and product for device %s." % device)
            return None

translator = None
joy_publisher = None
joy_subscriber = None


def joy_cb(joy):
    # if the translator doesn't exist, just republish the joy as we got it
    if translator is not None:
        joy = translator.translate(joy)

    joy_publisher.publish(joy)


class JoyTranslator(object):  # TODO fallback relay mode
    def __init__(self, config):
        self.config = config

        self.translation_vars = {
            'a': [],  # the current message's axes
            'b': [],  # the current message's buttons
            'pa': [],  # the previous message's axes
            'pb': [],  # the previous message's buttons
            'vars': {}  # misc variables that can be used by the configs
        }

        if 'init' in self.config.keys():
            for section in self.config['init'].keys():
                rospy.loginfo("Initializing section %s" % section)
                exec(self.config['init'][section], self.translation_vars)

	self.deadzone = 0.05
	if "deadzone" in config.keys():
		self.deadzone = float(config["deadzone"])


    def translate(self, joy):
        '''
        :param Joy joy:
        '''

        if len(joy.axes) != self.config['num_axes']:
            rospy.logerr("The received message has %i axes, but the config requires %i of them" % (
                len(joy.axes), self.config['num_axes']))
            return

        if len(joy.buttons) != self.config['num_buttons']:
            rospy.logerr("The received message has %i buttons, but the config requires %i of them" % (
                len(joy.buttons), self.config['num_buttons']))
            return

        self.translation_vars['pa'] = self.translation_vars['a']
        self.translation_vars['pb'] = self.translation_vars['b']
        self.translation_vars['a'] = joy.axes
        self.translation_vars['b'] = joy.buttons

        if 'callbacks' in self.config.keys():
            for section in self.config['callbacks'].keys():
                rospy.logdebug("Calling callback %s" % section)
                exec(self.config['callbacks'][section], self.translation_vars)

        result = Joy()
        result.header = joy.header
        result.axes = [0.] * 8
        result.buttons = [0] * 12

        for section in ('buttons', 'axes'):
            if section in self.config.keys():
                for item in self.config[section].keys():
                    try:
                        getattr(result, section)[item] = eval(self.config[section][item], self.translation_vars)
                    except Exception, e:
                        print "Evaluation of expression '%s' failed with exception:" % self.config[section][item]
                        traceback.print_exc()

	# we implement our own deadzone config to be able to change it on the fly
	for i in range(len(result.axes)):
		if -self.deadzone < result.axes[i] < self.deadzone:
			result.axes[i] = 0.0

        return result


if __name__ == '__main__':
    try:
        rospy.init_node('joy_translator')

        joy_publisher = rospy.Publisher("joy_translated", Joy, queue_size=10)
        joy_subscriber = rospy.Subscriber("joy", Joy, joy_cb, queue_size=10)

        # this loop constantly checks for the joystick, and if it starts being present, the translator searches for a
        # suitable config and starts translating; this allows to exchange joystick types without restarting the
        # translator

        while not rospy.is_shutdown():
            try:
                device = rospy.get_param("~dev", default="/dev/input/js0")
            except gaierror as e:  # this exception is sometimes thrown in the real missions on tds-*
                device = ""
                rospy.logwarn("Could not read parameter ~dev, ROS master connection is buggy")
                traceback.print_exc()

            if os.path.exists(device):
                if translator is None:
                    rospy.loginfo("Probing device %s" % device)
                    config_file = find_config_for_device(device)
                    if config_file is not None:
                        try:
                            with open(config_file, 'r') as stream:
                                try:
                                    config = yaml.load(stream)
                                    if config is not None:
                                        translator = JoyTranslator(config)
                                        rospy.loginfo("Started translating joystick %s configured by %s" % (device, config_file))
                                except yaml.YAMLError as e:
                                    rospy.logerr("Failed parsing config %s for device %s" % (config_file, device))
                                    traceback.print_exc()
                        except IOError as e:
                            rospy.logerr("Failed reading config %s for device %s" % (config_file, device))
                            traceback.print_exc()
                    else:
                        rospy.logerr("No config found for device %s" % device)
            else:
                if translator is not None:
                    translator = None
                    rospy.logerr("Device %s disappeared, switching to dumb relay mode" % device)

            rospy.rostime.wallsleep(0.5)

    except rospy.ROSInterruptException:
        pass
