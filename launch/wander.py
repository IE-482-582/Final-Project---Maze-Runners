#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
	global g_range_ahead
	#g_range_ahead = min(msg.ranges)
	range_l = msg.ranges
	print(len(range_l),min(range_l[0:319]))
	sec_1 = min(range_l[0:319])
	sec_2 = min(range_l[320:639],2)
	
	#~ if sec_1 <= 1 and sec_2 >= 1 and sec_3 >= 1 and sec_4 >= 1:
		#~ g_range_ahead = 0.5 
	#~ elif sec_1 >= 1 and sec_2 >= 1 and sec_3 >= 1 and sec_4 >= 1: 
		#~ g_range_ahead = 0.7
	#~ elif sec_1 <= 1 and sec_2 <= 1 and sec_3 >= 1 and sec_4 >= 1: 
		#~ g_range_ahead = 0.8
	#~ elif sec_1 >= 1 and sec_2 <= 1 and sec_3 >= 1 and sec_4 >= 1: 
		#~ g_range_ahead = 0.6
	#~ elif sec_1 >= 1 and sec_2 >= 1 and sec_3 <= 1 and sec_4 <= 1: 
		#~ g_range_ahead = 0.2
	#~ elif sec_1 >= 1 and sec_2 >= 1 and sec_3 <= 1 and sec_4 >= 1: 
		#~ g_range_ahead = 0.3
	print("Grange_value_all",sec_1)
	if sec_1 <=1.5:
		g_range_ahead = 0.5
	#~ if sec_2 <= 0.5:
		#~ g_range_ahead = 0.3
	#~ if sec_1 <= 0.5 and sec_2 <= 0.5: 
		#~ g_range_ahead = 0.2
	elif sec_1 >=2:
		g_range_ahead = 1
	print("grange_ahead",g_range_ahead)
		
g_range_ahead = 1 # anything to start

rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)
twist = Twist()
while not rospy.is_shutdown():
	scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
	cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	print("HI")
	if driving_forward:
		if (g_range_ahead == 0.5): #or rospy.Time.now() > state_change_time):
			print("hi12")
			twist.linear.x = 0
			twist.angular.z = 1
			obs = True
			obsta(obs)
			print("Obstacle Ahead") 
		elif (g_range_ahead == 0.3): #or rospy.Time.now() > state_change_time):
			twist.linear.x = 0.15
			twist.angular.z = -0.3
			print("Obstacle side")
		elif (g_range_ahead == 0.2): #or rospy.Time.now() > state_change_time):
			twist.linear.x = 0.0
			twist.angular.z =  0.3
			print("Obstacle everywhere")
			state_change_time = rospy.Time.now() + rospy.Duration(5)
		else:
			twist.linear.x = 0.2
			print("driving forward")
			cmd_vel_pub.publish(twist)

def obsta(obs):
	twist = Twist()
	i = 0
	if obs:
		while i <= 15:
			twist.angular.z = 0.25
			obs = False
			cmd_vel_pub.publish(twist)
			i += 1
		

	


