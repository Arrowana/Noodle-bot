#!/usr/bin/env python
import rospy
import robot_msgs.msgs
import math as m
from tf.transformations import euler_from_quaternion

heading_goal=0.

def imuCallback(imu_msg):
	roll, pitch, yaw=euler_from_quaternion(imu_msg.orientation)
	print "yaw", yaw, 180.*yaw/m.pi
	
	corr=180.*yaw/m.pi - heading_goal


if __name__ == '__main__':
	rospy.init_node('Driving_straight')

	rospy.Subscriber("imu", Imu , self.imuCallback)

	pub = rospy.Publisher('msg_teleops', PlatformCmd, queue_size=10)