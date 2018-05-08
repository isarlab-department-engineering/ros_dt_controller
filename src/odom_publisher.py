#!/usr/bin/env python
from __future__ import print_function
import gopigo3
import rospy, sys, time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32



def odom_publisher():
	rospy.init_node('odom_publisher', anonymous=True)
	GPG = gopigo3.GoPiGo3()






	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")



if __name__ == '__main__':
	odom_publisher()

