#!/usr/bin/env python

import os
import rospy

import random
from std_msgs.msg import Float32

'''
Description: Publishes a random number between 0 and 10.

File name: simple_publisher.py
Node name: simple_publisher
Published topic names: my_random_float
Message type: Float32
Subscriptions: None
Publish rate: 20hz
'''

#  ---Global Variables--  #

FILE_NAME =       "simple_publisher.py"
NODE_NAME =       "simple_publisher"
TOPIC_NAME =      "my_random_float"
MESSAGE_TYPE =    Float32
SUBSCRIPTIONS =   None 
PUBLISH_RATE_HZ = 20
# Change when done debugging
LOG_LEVEL =       rospy.DEBUG


def simple_publisher():
	
	#  Asserts name requirement of lab
	assert os.path.basename(__file__) == FILE_NAME 
	
	#  Publishing to the topic specified above using message type specified above with queue size of 10
	pub = rospy.Publisher(TOPIC_NAME, MESSAGE_TYPE, queue_size = 10)
	
	#  Initializes a node with name specified above to log specified above
	rospy.init_node(NODE_NAME, log_level = LOG_LEVEL)
	
	#  Publishes at rate specified above
	rate = rospy.Rate(PUBLISH_RATE_HZ)

	#  Logs node startup completion
	rospy.loginfo("%s started. Spinning..." % NODE_NAME)
	
	while not rospy.is_shutdown():
		
		#  Picks a random float between 0 and 10
		num = random.uniform(0.0, 10.0)
		
		#  Publishes random number
		pub.publish(num)
		
		#  Logs random number
		rospy.logdebug("Random generated number: %9f" % num)
		
		rate.sleep()

if __name__ == '__main__':
	
	try:
		
		simple_publisher()
	
	except rospy.ROSInterruptException:
		
		#  Message displayed upon keyboard interrupt
		rospy.loginfo("Interrupt recieved. Exiting %s" % NODE_NAME)

