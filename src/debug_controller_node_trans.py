#!/usr/bin/env python

#
# This script allows control over a raspberry pi based robot
# reads Twist messages from the controller topic and decodes it
# to move the motors using Adafruit libraries
#
# script by Andrea Fioroni - andrifiore@gmail.com
# GitHub repo: https://github.com/isarlab-department-engineering/ros-joy-controller/tree/master
#

## NOTE. DEBUG VERSION USES BOTH X AND Y
## NEED TO FIX THIS ASAP

import rospy,sys,atexit,time
import RPi.GPIO as gpio
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

class motor_driver:

    # LED PIN configuration
    FRONT     = 21
    REAR      = 20

    def atExitFunction(self):
        self.turnOffMotors()
        self.turnOffLights()

    def turnOffMotors(self):
        # log stop info
        rospy.loginfo("Turn off motors Turn on stop lights")
        # turn off motors
        self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        # turn on stop lights
        gpio.output(self.REAR,gpio.HIGH)
        # turn off front lights
        gpio.output(self.FRONT,gpio.LOW)

    def turnOffLights(self):
        gpio.output(self.REAR,gpio.LOW)
        gpio.output(self.FRONT,gpio.LOW)

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
	if self.rightSpeed < 40:
		self.rightSpeed = 40
	if self.leftSpeed < 40:
		self.leftSpeed = 40

    def setMotorSpeed(self):
        rospy.loginfo("Set motor speed Turn on front lights")
	rospy.loginfo("Left: "+str(self.leftSpeed)+" ;right: "+str(self.rightSpeed))
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
        # turn off stop lights
        gpio.output(self.REAR,gpio.LOW)
        # turn on front lights
        gpio.output(self.FRONT,gpio.HIGH)

    def __init__(self):
        # motor HAT setup
        self.mh = Adafruit_MotorHAT(addr=0x60) # setup Adafruit Motor HAT on 0x60 address
        # setup 2 motors
        self.mLeft = self.mh.getMotor(1) # left motor
        self.mRight = self.mh.getMotor(2) # right motor
        # speed vars
        self.leftSpeed = 0
        self.rightSpeed = 0
        # dir vars (1 = move forward, -1 = move backward)
        self.leftDir = 1
        self.rightDir = 1
        # set up and initialize led 
        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)
        gpio.setup(self.FRONT,gpio.OUT)
        gpio.setup(self.REAR,gpio.OUT)
        # subscribe ros topic
        rospy.Subscriber("cmd_vel", Twist, self.callback) # subscribe to cmd_vel topic
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
    rospy.init_node('ros_motor_driver', anonymous=True) # create a ros_motor_driver node
    try:
	    rospy.spin() # loop until shutdown
    except KeyboardInterrupt:
	    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
