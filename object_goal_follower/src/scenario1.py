#!/usr/bin/env python
# map: scenario1.py

from __future__ import print_function
import sys
import rospy
import cv2
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import*
from math import radians, degrees, pi, sin, cos, pow, sqrt, atan
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from copy import deepcopy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tf

from std_msgs.msg import UInt16

MAX_LIN_X = 0.22
MAX_ANG_Z = 2.82
lin_x=MAX_LIN_X/2
ang_z=MAX_ANG_Z/2

start_pos=[0,0,0,1.0]

R_pos=[0.245437295871,0.010955891418,0.0000000000,0.999827705321]

goal_R_pos=[-0.117758251717,-1.64810655851, -0.720717552191, 0.693228829438]
#y_pos
Y_pos=[0.30606096869,-0.457031596039,0.319710215833,0.947515370795]

goal_Y_pos=[0.323481684037,-1.70521210514, -0.47578966526, 0.879559090927]

class GoalPose:
	x=0.
	y=0.
	theta=0.
	z=0.
	w=0.

def callback(msg):
	current_pose=msg

def grap(servo_pub):
	print("grap")					
	servo_pub.publish(170)
	rospy.sleep(1)

def release(servo_pub):
	print("release")
	servo_pub.publish(10)
	rospy.sleep(1)

def up(servo2_pub):
	print("up")
	servo2_pub.publish(140)
	rospy.sleep(1)#(sec)

def down(servo2_pub):
	print("down")
	servo2_pub.publish(15)
	rospy.sleep(1)#(sec)

def move_to(goal_point,ac):
	
	#wait=0
	#while (wait==0):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map" #'map'
		goal.target_pose.header.stamp = rospy.Time.now()
	
		goal.target_pose.pose.orientation.x=0
		goal.target_pose.pose.orientation.y=0
		goal.target_pose.pose.orientation.z=goal_point.z
		goal.target_pose.pose.orientation.w=goal_point.w

		goal.target_pose.pose.position.x=goal_point.x
		goal.target_pose.pose.position.y=goal_point.y
		goal.target_pose.pose.position.z=0
		print (goal)
		ac.send_goal(goal)
		wait=ac.wait_for_result()

		if not wait:
			print("error!2")
			
			#rospy.logerr("fail")
			#rospy.signal_shutdown("fail")
		else:
			rospy.sleep(1)

def get_dist(x,y):
	return sqrt(pow(abs(x), 2) + pow(abs(y), 2))


def linear_mov(x,y,cmd_vel_pub):
	
	msg=Twist()

	#initialize
	msg.angular.x = msg.angular.y = msg.angular.z = 0.0
	msg.linear.x = msg.linear.y  = msg.linear.z  = 0.0

	#compute dist
	dist=get_dist(x,y)	
	
	#set speed
	if(x>=0):
		msg.linear.x=lin_x
	elif(x<0):
		msg.linear.x=-lin_x

	#compute time
	t_move=dist/lin_x	
	t_end=rospy.Time.now()+rospy.Duration(t_move)
	
	#executing	
	print("backward")
	while(rospy.Time.now()<t_end):			
		cmd_vel_pub.publish(msg)
		rospy.sleep(0.01)
	
	#stop
	msg.linear.x = 0	
	cmd_vel_pub.publish(msg)
	rospy.sleep(1)

def get_angle(x, y):
	if(x>=0 and y>=0):	#case1
		return atan(abs(y)/abs(x))
	elif(x>=0 and y<0):	#case2
		return -atan(abs(y)/abs(x))
	elif(x<0 and y<0): 	#case3
		return -(pi-atan(abs(y)/abs(x)))
	elif(x<0 and y>=0):	#case4
		return pi-atan(abs(y)/abs(x))

def turn(x,y,cmd_vel_pub):
	msg=Twist()

	#initialize
	msg.angular.x = msg.angular.y = msg.angular.z = 0.0
	msg.linear.x = msg.linear.y  = msg.linear.z  = 0.0

	#compute angle
	angle=get_angle(x,y)

	#set direction of turn
	if(angle<0):		
		wise=-1	#cw
	else:
		wise=1	#ccw	
	angle=abs(angle)

	#set speed
	msg.angular.z=ang_z*wise

	#compute time
	t_turn=angle/ang_z	
	t_end=rospy.Time.now()+rospy.Duration(t_turn)
		
	#executing
	print("turn")	
	while(rospy.Time.now()<t_end):
		print("turning")				
		cmd_vel_pub.publish(msg)
		rospy.sleep(0.01)

	#stop
	msg.angular.z=0.0
	cmd_vel_pub.publish(msg)
	rospy.sleep(1)

def object_detect(cmd_vel_pub, servo_pub, servo2_pub):
	#image process	
	img_topic = "/camPi/image_raw" #"/camera/rgb/image_raw"	
	image_sub = rospy.Subscriber(img_topic, Image, callback)
	cx=0
	cy=0
	bridge = CvBridge()
	image_received = False
	
	



	

if __name__ == '__main__':
	rospy.init_node('scenario1')
	cmd_vel_pub=rospy.Publisher('cmd_vel', Twist, queue_size=10)
	servo_pub=rospy.Publisher('servo', UInt16, queue_size=10)
	servo2_pub=rospy.Publisher('servo2', UInt16, queue_size=10)
	#odom_sub=rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
	
	ac=actionlib.SimpleActionClient('move_base', MoveBaseAction)
	ac.wait_for_server()#check
	
	#initialize
	print("initialize")
	down(servo2_pub)
	release(servo_pub)
	#turn(-1,0,cmd_vel_pub)
	#turn(-1,0,cmd_vel_pub)
			

	try:
		while not rospy.is_shutdown():
			print("Cargo classify")

			#basic UI
			print("start: 1/quit: 0")
			cmd=input(":")			
			if(cmd==1):
				print("start")
			else:
				print("quit")
				ac.cancel_goal()				
				exit()
			
			#start
			start=GoalPose()
			[start.x, start.y, start.z, start.w]=start_pos

			#R_pose
			R_pose=GoalPose()
			[R_pose.x, R_pose.y, R_pose.z, R_pose.w]=R_pos

			#goal_point_R
			goal_point_R=GoalPose()
			[goal_point_R.x, goal_point_R.y, goal_point_R.z, goal_point_R.w]=goal_R_pos
			
			Y_pose=GoalPose()
			[Y_pose.x, Y_pose.y, Y_pose.z, Y_pose.w]=Y_pos
		
			goal_point_Y=GoalPose()
			[goal_point_Y.x, goal_point_Y.y, goal_point_Y.z, goal_point_Y.w]=goal_Y_pos
			
			#start->R_pose->goal_point_R->Y_pose0->goal_point_y->start
						
			#1--go to Red object grip-up
			print("go")
			move_to(R_pose,ac)
			linear_mov(0.25,0,cmd_vel_pub)
			grap(servo_pub)
			up(servo2_pub)
			
			move_to(start,ac)
			
			#2--go to goal_point_R down-release
			move_to(goal_point_R,ac)	
			down(servo2_pub)
			release(servo_pub)
			linear_mov(-0.1,0,cmd_vel_pub)			

			#3--go to Y_pose
			move_to(Y_pose,ac)
			linear_mov(0.23,0,cmd_vel_pub)
			grap(servo_pub)
			up(servo2_pub)
			linear_mov(-0.1,0,cmd_vel_pub)

			#4--go to goal_point_Y down-release
			move_to(goal_point_Y,ac)			
			down(servo2_pub)
			release(servo_pub)
			linear_mov(-0.1,0,cmd_vel_pub)

			#5--go to start
			move_to(start,ac)

		
			print("finish")
	except:
		ac.cancel_goal() 
		print("error!")














