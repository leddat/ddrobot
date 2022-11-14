#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

rospy.init_node('vel_Publisher')
pub1 = rospy.Publisher('/dd_robot/joint_left_wheel/command', Float64, queue_size=1)
pub2 = rospy.Publisher('/dd_robot/joint_right_wheel/command', Float64, queue_size=1)

rate=rospy.Rate(2)

vel = Float64()
vel1 = Float64()
vel2 = Float64()

vel.data = 20
vel1.data = -10
vel2.data = -10

while not rospy.is_shutdown():
	pub.publish(vel)
	pub1.publish(vel1)
	pub2.publish(vel2)
	
	rate.sleep()



