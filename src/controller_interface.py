#!/usr/bin/env python

#
# This script allows control over a raspberry pi based robot
# reads Twist messages from the controller topic and decodes it
# to move the motors using Adafruit libraries
#
# script by Andrea Fioroni - andrifiore@gmail.com
# GitHub repo: https://github.com/isarlab-department-engineering/ros-joy-controller/tree/master
#

import rospy,sys,atexit,time
import RPi.GPIO as gpio
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

class motor_driver:

    # LED PIN configuration
    FRONT_1     = 21
    FRONT_2     = 20
    REAR_1      = 19
    REAR_2      = 16

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
        gpio.output(self.REAR_1,gpio.HIGH)
        gpio.output(self.REAR_2,gpio.HIGH)
        # turn off front lights
        gpio.output(self.FRONT_1,gpio.HIGH)
        gpio.output(self.FRONT_2,gpio.HIGH)

    def turnOffLights(self):
        gpio.output(self.REAR_1,gpio.HIGH)
        gpio.output(self.REAR_2,gpio.HIGH)
        gpio.output(self.FRONT_1,gpio.LOW)
        gpio.output(self.FRONT_2,gpio.LOW)

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

    def setMotorSpeed(self):
        rospy.loginfo("Set motor speed Turn on front lights")
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
        gpio.output(self.REAR_1,gpio.LOW)
        gpio.output(self.REAR_2,gpio.LOW)
        # turn on front lights
        gpio.output(self.FRONT_1,gpio.LOW)
        gpio.output(self.FRONT_2,gpio.LOW)

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
        gpio.setup(self.FRONT_1,gpio.OUT)
        gpio.setup(self.REAR_1,gpio.OUT)
        gpio.setup(self.FRONT_2,gpio.OUT)
        gpio.setup(self.REAR_2,gpio.OUT)
        # subscribe ros topic
        rospy.Subscriber("cmd_vel", Twist, self.callback) # subscribe to cmd_vel topic
	rospy.loginfo("Initialized controller class")

    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + " Incoming Twist Message")
        # converts angular + linear values from cmd_vel
        # to 2 speed values for the motors
        #
        # we only need two of the six available values
        # so input Twist message will always be:
        # [ L X X ] [ X X A ]
        # with L representing the linear x-axis velocity
        # A representing the angular z-axis velocity
        # and X are values we don't need here
        #
        # good kown values are L = 150 and A = 10 -> 30 ( +/- )
        # (-10 = turn left, 0 = go straight, 10 = turn right)
        # angular 30 and linear 150 will make the robot turn with
        # only one wheel, while the other motor will be at 0 speed
        # distance between the two wheels of the robot (cm)
        WHEEL_DIST = 10 #TODO misure this param
        # wheel radius, not needed right now
        # but could be useful for a more accurate control
        # WHEEL_RADIUS = 2.5
        # change this to change the curve arc
        # modelling the difference on the two speeds
        diffParam = 2.0
        velDiff = (WHEEL_DIST * data.angular.z) / diffParam;
        if(data.linear.x < 0): # moving backward
            velDiff = -velDiff # reverse the curve arc
        self.leftSpeed = (data.linear.x + velDiff) #/ WHEEL_RADIUS
        self.rightSpeed = (data.linear.x - velDiff) #/ WHEEL_RADIUS
        self.speedControl()
        if data.linear.x == 0 and data.angular.z == 0:
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
