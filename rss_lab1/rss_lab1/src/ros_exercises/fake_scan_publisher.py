#!/usr/bin/env python

import rospy
import os
import numpy as np
import random

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class fakeScanPublisher:

	#  ---Global Variables---  #

	_FILE_NAME =          "fake_scan_publisher.py"
	_NODE_NAME =          "fake_scan_publisher"
	_TOPIC_NAME =         "fake_scan"
	_MESSAGE_TYPE =       LaserScan
	_SUBSCRIPTIONS =      None
	_PUBLISH_RATE_HZ =    20
	_QUEUE_SIZE =         10

	#  ---LaserScan Parameters---  #

	_LS_FRAME_ID =        "base_link"
	_LS_ANGLE_MIN =       -2.0/3.0 * np.pi
	_LS_ANGLE_MAX =       2.0/3.0 * np.pi
	_LS_ANGLE_INCREMENT = 1.0/300.0 * np.pi
	_LS_TIME_INCREMENT =  (1/20.0)
	_LS_RANGE_MAX =       10.0
	_LS_RANGE_MIN =       1.0
	_LS_INTENSITIES =     None

	_LOG_LEVEL =          rospy.DEBUG

	def __init__(self):

		#  Initializes a node with name specified above to log specified above
		rospy.init_node(self._NODE_NAME, log_level = self._LOG_LEVEL)

		#  Logs node startup completion
		rospy.loginfo("%s started. Spinning..." % self._NODE_NAME)

		#  Defines the parameters as specified by the lab
		pubs_topic = rospy.get_param("~pubs_topic", self._TOPIC_NAME)                                             
		pubs_rate = rospy.get_param("~pubs_rate", self._PUBLISH_RATE_HZ)
		angle_min = rospy.get_param("~angle_min", self._LS_ANGLE_MIN)
		angle_max = rospy.get_param("~angle_max", self._LS_ANGLE_MAX)
		angle_increment = rospy.get_param("~angle_increment", self._LS_ANGLE_INCREMENT)
		range_min = rospy.get_param("~range_min", self._LS_RANGE_MIN)
		range_max = rospy.get_param("~range_max", self._LS_RANGE_MAX)

		#  Asserts name requirement of lab
		assert os.path.basename(__file__) == self._FILE_NAME
		assert angle_max > angle_min
		assert range_max > range_min
		
		#  Publishing to the topic using the message and queue size all specified above
		pub = rospy.Publisher(pubs_topic, self._MESSAGE_TYPE, queue_size = self._QUEUE_SIZE)

		#  Publishes at rate specified above
		rate = rospy.Rate(pubs_rate)

		#  Sets message type as specified above
		laser = LaserScan()

		#  Sets message properties that don't change every refresh
		laser.header.frame_id = self._LS_FRAME_ID
		laser.angle_min = angle_min
		laser.angle_max = angle_max
		laser.angle_increment = angle_increment
		laser.range_min = range_min
		laser.range_max = range_max

		#  Finds length of necessary array
		laser.ranges = [0] * int(round((angle_max - angle_min)/angle_increment))

		while not rospy.is_shutdown():

			laser.header.stamp = rospy.Time.now()
			laser.scan_time = rospy.get_time()
			
			#  Populates ranges with random data
			for i in range(len(laser.ranges)):
				laser.ranges[i] = random.uniform(range_min, range_max)

			#  Publishes generated laser data
			pub.publish(laser)

			#  Logs generated laser laser data
			rospy.logdebug("Generated laser data")
			
			rate.sleep()


if __name__ == '__main__':
	
	try:
		
		node = fakeScanPublisher()
	
	except rospy.ROSInterruptException:
		
		#  Message displayed upon keyboard interrupt
		rospy.loginfo("Interrupt recieved. Exiting %s" % NODE_NAME)