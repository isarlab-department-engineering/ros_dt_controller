#!/usr/bin/env python
from __future__ import print_function
import easygopigo3 as easy
import rospy, sys, time
import numpy as np
import math
from math import sin, cos, pi
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Int32

wheel_radius = 0.033 # radius of wheels
wheel_distance = 0.120 # distance between wheels

def odom_publisher():
	global wheel_radius, wheel_distance
	rospy.init_node('odom_publisher', anonymous=True)
	GPG = easy.EasyGoPiGo3()
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
	odom_broadcaster = tf.TransformBroadcaster()
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()
	last_theta = 0
	last_x = 0
	last_y = 0
	last_deg_left, last_deg_right = GPG.read_encoders()
	r = rospy.Rate(10.0)
	try:
		while not rospy.is_shutdown():
			current_time = rospy.Time.now()
			current_deg_left, current_deg_right = GPG.read_encoders()
			# compute odometry in a typical way given the velocities of the robot
			dt = (current_time - last_time).to_sec()
			delta_deg_left = current_deg_left - last_deg_left
			delta_deg_right = current_deg_right - last_deg_right
			vel_left = (np.radians(delta_deg_left)/dt)*wheel_radius
			vel_right = (np.radians(delta_deg_right)/dt)*wheel_radius
			# calculate x-y-theta velocity
			vel_theta = (vel_right - vel_left) / wheel_distance
			current_theta = (vel_theta * dt) + last_theta
			vx = ((vel_right + vel_left)/2)*cos(current_theta)
			vy = ((vel_right + vel_left)/2)*sin(current_theta)
	    		x = last_x + vx * dt
			y = last_y + vy * dt
			# since all odometry is 6DOF we'll need a quaternion created from yaw
			odom_quat = tf.transformations.quaternion_from_euler(0, 0, current_theta)
			# send transform base_link odom
			odom_broadcaster.sendTransform(
			(x, y, 0.),
			odom_quat,
			current_time,
			"base_link",
			"odom"
			)
			# next, we'll publish the odometry message over ROS
			odom = Odometry()
			odom.header.stamp = current_time
			odom.header.frame_id = "odom"
			# set the position
			odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
			# set the velocity
			odom.child_frame_id = "base_link"
			odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vel_theta))
			# publish the message
			print(odom)
			odom_pub.publish(odom)
			# update variables
			last_time = current_time
			last_deg_left = current_deg_left
			last_deg_right = current_deg_right
			last_theta = current_theta
			last_x = x
			last_y = y
			r.sleep()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	odom_publisher()

