#!/usr/bin/env python

import rospy
import os
import sys

import numpy as np
from std_msgs.msg import Float32

'''
Description: Subscribes to the topic published by the simple_publisher and publishes the natural log of the received messages.

File name: simple_subscriber.py
Node name: simple_subscriber
Publish topic names: random_float_log
Message type: Float32
Subscription topics names: my_random_float
'''

#  ---Global Variables--- #

FILE_NAME =     "simple_subscriber.py"
NODE_NAME =     "simple_subscriber"
TOPIC_NAME =    "random_float_log"
MESSAGE_TYPE =  Float32
SUBSCRIPTIONS = "my_random_float"
#  Change when done debugging
LOG_LEVEL =     rospy.DEBUG


def callback(data):
	
	#  Assigns a variable to the recieved data and asserts it is not negative
	message = data.data
	assert(message >= 0.0)
	
	#  Logs message recieved from subscription specified above
	rospy.logdebug("Recieved %1.0f from topic %s" % (message, SUBSCRIPTIONS))

	#  Ensures log of zero is never taken
	if message < sys.float_info.min:
		message = sys.float_info.min

	#  Takes natural log of message
	result = np.log(data.data)
	
	#  Publishes result of log	
	rospy.Publisher(TOPIC_NAME, MESSAGE_TYPE, queue_size = 10).publish(result)

	#  Logs the published result
	rospy.logdebug("Random log: %9f" % result)
	

def simple_subscriber():
	
	#  Asserts name requirement of the lab
	assert os.path.basename(__file__) == FILE_NAME
	
	#  Initializes node with name specified above
	rospy.init_node(NODE_NAME, log_level = LOG_LEVEL)

	#  Specifies subscriptions and message type as per abvoe
	rospy.Subscriber(SUBSCRIPTIONS, MESSAGE_TYPE, callback)

	#  Logs node startup completion
	rospy.loginfo("%s started. Spinning..." % NODE_NAME)
	
	self.values = []

	#  Continues node until shut down
	rospy.spin()

if __name__ == '__main__':
	
	simple_subscriber()
	
	
