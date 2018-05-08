#!/usr/bin/env python

#
# This script allows control over a raspberry pi based robot
# reads Twist messages from the controller topic and decodes it
# to move the motors using GoPiGo3 libraries
#



import rospy,sys,atexit,time
import easygopigo3 as easy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class motor_driver:

    def atExitFunction(self):
        self.turnOffMotors()

    def turnOffMotors(self):
        # log stop info
        rospy.loginfo("Turn off motors Turn on stop lights")
        # turn off motors
        self.gpg.stop()


    def speedControl(self):

	if not (self.leftSpeed < 0 and self.rightSpeed < 0):
            if(self.leftSpeed == self.rightSpeed):
	        self.dir = 0
	    else:
	        if (self.leftSpeed > self.rightSpeed):
	            self.dir = 1
	        else:
	            self.dir = 2
	        
		

    def setMotorSpeed(self):
        rospy.loginfo("Set motor speed Turn on front lights")
        
        if(self.dir == 0): # move motor forward
            self.gpg.forward()
        elif(self.dir == 1): # move motor right
            self.gpg.right()
        elif(self.dir == 2): # move motor left
            self.gpg.left()
        else: # move motor backward
            self.gpg.backward()


    def __init__(self):
        # motor setup
        self.gpg = easy.EasyGoPiGo3() # GoPiGo3 Motor

        rospy.Subscriber("cmd_vel", Twist, self.callback, queue_size=1) # subscribe to cmd_vel topic
	rospy.loginfo("Initialized controller class")

    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + " Incoming Twist Message")
        self.leftSpeed = data.linear.x
        self.rightSpeed = data.linear.y
	self.dir = 3
        self.speedControl()
        if data.linear.x == 0 and data.linear.y == 0:
            self.turnOffMotors()
        else:
            self.setMotorSpeed()

def main(args):
    m_driver = motor_driver()
    @atexit.register
    def atExitClass():
    	m_driver.atExitFunction()
    rospy.init_node('ros_motor_driver', anonymous=True) # create a ros_motor_driver node
    try:
	    rospy.spin() # loop until shutdown
    except KeyboardInterrupt:
	    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
