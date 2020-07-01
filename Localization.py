#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray

#Defining a class
class Marker_detect():

	def __init__(self):
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.whycon_marker = {}	# Declaring dictionaries
	
		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic

	# Callback for /whycon/poses
	# Please fill in the function
	def whycon_data(self,msg):
		
		#Getting markers position with round-off upto 4 digits 
		roundoff=4

		for count in range(len(msg.poses)):
			self.whycon_marker[count]=[round(msg.poses[count].position.x,roundoff),round(msg.poses[count].position.y,roundoff),round(msg.poses[count].position.z,roundoff)]
		

		# Printing the detected markers on terminal
		print(self.whycon_marker)


if __name__=="__main__":
	marker = Marker_detect()
	while not rospy.is_shutdown():
		rospy.spin()