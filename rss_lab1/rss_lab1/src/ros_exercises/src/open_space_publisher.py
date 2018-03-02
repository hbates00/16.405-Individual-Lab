#!/usr/bin/env python

import rospy
import os

from ros_exercises.msg import OpenSpace
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class openSpacePublisher:

	#  ---Global Variables---  #

	_FILE_NAME =          "open_space_publisher.py"
	_NODE_NAME =          "open_space_publisher"
	_TOPIC_NAME =         "open_space"
	_MESSAGE_TYPE =       OpenSpace
	_QUEUE_SIZE =         10
	_SUBSCRIPTIONS =      "fake_scan"
	_SUBSCRIPTION_TYPES = LaserScan

	_LOG_LEVEL =          rospy.INFO
	

	def __init__(self):

		#  Asserts name requirement of the lab
		assert os.path.basename(__file__) == self._FILE_NAME
		
		#  Initializes node with name specified above
		rospy.init_node(self._NODE_NAME, log_level = self._LOG_LEVEL)

		self.subs_topic = rospy.get_param("~subs_topic", self._SUBSCRIPTIONS)
		self.pub_topic = rospy.get_param("~pub_topic", self._TOPIC_NAME)

		#  Specifies subscriptions and message type as per abvoe
		rospy.Subscriber(self.subs_topic, self._SUBSCRIPTION_TYPES, self.callback)

		#  Logs node startup completion
		rospy.loginfo("%s started. Spinning..." % self._NODE_NAME)

		#  Continues node until shut down
		rospy.spin()


	def callback(self, laser):
		
		(mx, angle) = self.get_max_coordinates(laser)

		#  Sets message type as specified above
		openspace = OpenSpace()

		#  Sets message properties
		openspace.distance = mx
		openspace.angle = angle

		#  Publishes result
		rospy.Publisher(self.pub_topic, self._MESSAGE_TYPE, queue_size = self._QUEUE_SIZE).publish(openspace)

		#  Logs the published result
		rospy.logdebug((mx, angle))

	def get_max_coordinates(self, laser):

		ind = laser.ranges.index(max(laser.ranges))
		angle = laser.angle_min + laser.angle_increment * ind	

		return (laser.ranges[ind], angle)
		
if __name__ == '__main__':
	
	try:
		
		node = openSpacePublisher()
	
	except rospy.ROSInterruptException:
		
		#  Message displayed upon keyboard interrupt
		rospy.loginfo("Interrupt recieved. Exiting %s" % self._NODE_NAME)