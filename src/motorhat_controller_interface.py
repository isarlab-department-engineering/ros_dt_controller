#!/usr/bin/env python

#
# This script allows control over a raspberry pi based robot
# reads Twist messages from the controller topic and decodes it
# to move the motors using Adafruit libraries
#
# GitHub repo: https://github.com/isarlab-department-engineering/ros_dt_controller
#

# IMPORTANT NOTE
# THIS SCRIPT ATM DOESN'T TAKE A "REAL" TWIST MESSAGE
# Twist/Linear/X&Y are the speed of the 2 motors 

import rospy,sys,atexit,time
import RPi.GPIO as gpio
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

class motor_driver:

    # motors pin
    left_motor = 1
    right_motor = 2

    def atExitFunction(self):
        self.turnOffMotors()

    def turnOffMotors(self):
        # log stop info
        rospy.loginfo("Turn off motors")
        # turn off motors
        self.mh.getMotor(self.left_motor).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(self.right_motor).run(Adafruit_MotorHAT.RELEASE)

    def speedControl(self):
        # check if l and r speed are in the -255 - 255 range
        # and set the direction vars, since Adafruit libraries
        # only allow us to set a positive speed value and then
        # to move the motors backward/forward
        if(self.leftSpeed >= 0):
            self.leftDir = 1
            if(self.leftSpeed > 255):
                self.leftSpeed = 255
        else:
            self.leftDir = -1
            self.leftSpeed = -self.leftSpeed
            if(self.leftSpeed > 255):
                self.leftSpeed = 255
        if(self.rightSpeed >= 0):
            self.rightDir = 1
            if(self.rightSpeed > 255):
                self.rightSpeed = 255
        else:
            self.rightDir = -1
            self.rightSpeed = -self.rightSpeed
            if(self.rightSpeed > 255):
                self.rightSpeed = 255
	# ADD CONTROL IF TOO SLOW 
	## FIRST IMPLEMENTATION
	if self.leftSpeed < 40:
		self.leftSpeed=40
	if self.rightSpeed < 40:
		self.rightSpeed=40

    def setMotorSpeed(self):
        rospy.loginfo("Set motor speed")
        self.mLeft.setSpeed(int(self.leftSpeed))
        self.mRight.setSpeed(int(self.rightSpeed))
        if(self.leftDir == 1): # move left motor forward
            self.mLeft.run(Adafruit_MotorHAT.FORWARD)
        else: # move left motor backward
            self.mLeft.run(Adafruit_MotorHAT.BACKWARD)
        if(self.rightDir == 1): # move right motor forward
            self.mRight.run(Adafruit_MotorHAT.FORWARD)
        else: # move right motor backward
            self.mRight.run(Adafruit_MotorHAT.BACKWARD)

    def __init__(self):
        # motor HAT setup
        self.mh = Adafruit_MotorHAT(addr=0x60) # setup Adafruit Motor HAT on 0x60 address
        # setup 2 motors
        self.mLeft = self.mh.getMotor(self.left_motor) # left motor
        self.mRight = self.mh.getMotor(self.right_motor) # right motor
        # speed vars
        self.leftSpeed = 0
        self.rightSpeed = 0
        # dir vars (1 = move forward, -1 = move backward)
        self.leftDir = 1
        self.rightDir = 1
        # subscribe ros topic
        rospy.Subscriber("cmd_vel", Twist, self.callback, queue_size=1) # subscribe to cmd_vel topic
        rospy.loginfo("Initialized controller class")

    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + " Incoming Twist Message")
        self.leftSpeed = data.linear.x
        self.rightSpeed = data.linear.y
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
    rospy.init_node('controller_interface', anonymous=True) # create a ros_motor_driver node
    try:
	    rospy.spin() # loop until shutdown
    except KeyboardInterrupt:
	    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
