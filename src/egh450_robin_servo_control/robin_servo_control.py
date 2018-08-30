#!/usr/bin/env python

from math import *

import rospy

from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Bool

class RobinServoControl():
	def __init__(self):
		# Set up a subscriber that will act as the trigger mechanism
		# Use a std_msgs/Bool to allow for open/close
		self.sub_deploy = rospy.Subscriber('~deploy', Bool, self.callback_deploy)

		# Set the PWM Override message so we don't have to re-compile it every time
		# By default, set it so that none of the other servo outputs are overriden
		self.msg_out = OverrideRCIn()
		for ch in self.msg_out.channels:
			ch = self.msg_out.CHAN_NOCHANGE

		# Read in what servo we want to control, and it's open/close settings
		self.servo_ch = int(rospy.get_param("~servo_ch"))
		self.servo_low = int(rospy.get_param("~servo_low"))
		self.servo_high = int(rospy.get_param("~servo_high"))

		# Set the starting value for the servo
		servo_start_high = bool(rospy.get_param("~servo_start_high"))
		if servo_start_high:
			self.servo_value = self.servo_high
		else:
			self.servo_value = self.servo_low

		# Set up the publisher that will send the output override commands
		self.pub_override = rospy.Publisher('~override', OverrideRCIn, queue_size=10)

		# Set up a timer to send PWM commands to the autopilot
		# By default, robin expects a command stream at a 5Hz, so lets double that
		self.timer = rospy.Timer(rospy.Duration(0.1), self.callback_pwm_out)

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_deploy.unregister()

	def callback_deploy(self, msg_in):
		# If the message value was set to True
		# Set the servo to high, otherwise set to low
		if msg_in.data:
			rospy.loginfo("[PAY] Set servo high!")
			self.servo_value = self.servo_high
		else:
			rospy.loginfo("[PAY] Set servo low!")
			self.servo_value = self.servo_low

	def callback_pwm_out(self, e):
		# Update the channel we're interested in with
		# our output value (-1 as channel is 1->8, but array is 0->7)
		self.msg_out.channels[self.servo_ch - 1] = self.servo_value

		# Publish the command message
		self.pub_override.publish(self.msg_out)






