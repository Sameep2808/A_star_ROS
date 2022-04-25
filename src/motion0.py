#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import Quaternion
from math import atan2
from nmap import *


def newOdom(msg):
	global x
	global y
	global roll, pitch, yaw
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y


if __name__ == "__main__" :
	m,am = create_map()
	global cll
	global rr
	start = []
	global goal
	start,goal,L,sa,ga=get_input(m,am)
	root = Node(start, sa, 0 , None, 0, start)
	F,C,O,Pxy = DS(root,goal,L,ga,m,am)
	p=reverse_path(F,m,am)
	Vig(O,Pxy,p,m,am)	
	# print(p)
	rospy.init_node("speed_controller")
	global x
	global y
	global yaw
	yaw = 0
	x = 0
	y = 0
	sub = rospy.Subscriber("/odom", Odometry, newOdom)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	speed = Twist()

	r = rospy.Rate(100)
	
	goal = Point()
	for point in p :
		goal_x = (point.d[0] - 500 )/100
		goal_y = (point.d[1] - 500 )/100
		inc_x,inc_y = 100,100
		rospy.loginfo(f"X : {goal_x}, Y : {goal_y}")

		while (inc_x > 0.01 and inc_y > 0.01):
			inc_x = (goal_x) - (x) 
			inc_y = (goal_y) - (y) 
			# rospy.loginfo(f"X : {x}, Y : {y}")
			angle_to_goal = atan2(inc_y, inc_x) 
			rospy.loginfo(f"T : {angle_to_goal - yaw}")

			if (angle_to_goal - yaw) > 0.3:
				speed.linear.x = 0.2
				speed.angular.z = 0.5
			elif (angle_to_goal - yaw) < -0.3:
				speed.linear.x = 0.2
				speed.angular.z = -0.5
			else:
				speed.linear.x = 0.4
				speed.angular.z = 0.0

			pub.publish(speed)
			r.sleep()

	speed.linear.x = 0.0
	speed.angular.z = 0.0  	
	pub.publish(speed)

