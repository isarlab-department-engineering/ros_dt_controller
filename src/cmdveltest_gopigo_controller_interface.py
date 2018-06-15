#!/usr/bin/env python

import rospy,atexit,sys
import tf.transformations
import easygopigo3 as easy
import numpy as np
from geometry_msgs.msg import Twist

class motor_driver: 
	
	# const
	WHEEL_RADIUS = 0.03325 # radius of wheels
	WHEEL_DISTANCE = 0.117 # distance between wheels
	# global var
	rightSpeed = 0	
	leftSpeed = 0
	
	def atExitFunction(self):
	        self.turnOffMotors()

	def turnOffMotors(self):
		# log stop info
		rospy.loginfo("Turn off motors Turn on stop lights")
		# turn off motors
		self.gpg.stop()		

	def callback(self,data):
		rospy.loginfo("Received a /cmd_vel message!")
		rospy.loginfo("Linear Components: [%f, %f, %f]"%(data.linear.x, data.linear.y, data.linear.z))
		rospy.loginfo("Angular Components: [%f, %f, %f]"%(data.angular.x, data.angular.y, data.angular.z))
		# Use the kinematics of your robot to map linear and angular velocities into motor commands
		vLeft = data.linear.x - (data.angular.z * self.WHEEL_DISTANCE / 2)
		vRight = data.linear.x + (data.angular.z * self.WHEEL_DISTANCE / 2)
		self.leftSpeed = 360* vLeft / (2 * np.pi * self.WHEEL_RADIUS)
    		self.rightSpeed = 360 * vRight / (2 * np.pi * self.WHEEL_RADIUS)
		rospy.loginfo("Left,Right speed: [%d, %d]"%(self.leftSpeed, self.rightSpeed))
		self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT,int(self.leftSpeed))
		self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT,int(self.rightSpeed))    

	def __init__(self):
		# motor setup
		self.gpg = easy.EasyGoPiGo3() # GoPiGo3 Motor
		rospy.Subscriber("cmd_vel", Twist, self.callback, queue_size=1) # subscribe to cmd_vel topic
		rospy.loginfo("Initialized controller class")

def main(args):
    m_driver = motor_driver()
    @atexit.register
    def atExitClass():
    	m_driver.atExitFunction()
    rospy.init_node('base_controller')
    try:
	    rospy.spin() # loop until shutdown
    except KeyboardInterrupt:
	    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
