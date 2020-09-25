#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion, PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import ColorRGBA, Header
import numpy

from visualization_msgs.msg import Marker

class FSM(object):
	"""docstring for FSM"""
	def __init__(self):
		rospy.init_node("FSM")
		self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
		self.cmdVelSub = rospy.Subscriber("/cmd_vel_state",TwistStamped,self.cmdVelCB)
		self.goalSub = rospy.Subscriber("/move_base_simple/goal",PoseStamped, self.goalCB)
		self.odomSub = rospy.Subscriber("/odom", Odometry, self.odomCB)
		self.position = None
		self.goalStamped = None
		self.velToPub = None
		self.rate = rospy.Rate(10)
		self.state = "obstacleavoid"

	def cmdVelCB(self,msg):
		if(msg.header.frame_id == self.state):
			self.velToPub = msg.twist

	def goalCB(self,msg):
		self.goalStamped = msg
		self.state = "obstacleavoid"

	def odomCB(self,msg):
		self.position = msg.pose.pose.position

	def checkStateTransition(self):
		if(self.distanceBetweenTwoPositions(self.position,self.goalStamped.pose.position) < 2.0):
			self.state = "personfollow"

	def distanceBetweenTwoPositions(self,p1,p2):
		return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)

	def main(self):
		while((not rospy.is_shutdown()) and (self.position == None or self.goalStamped == None or self.velToPub == None)):
			rospy.loginfo("No data yet")
		while(not rospy.is_shutdown()):
			self.checkStateTransition()
			self.pub.publish(self.velToPub)
			self.rate.sleep()



if __name__ == '__main__':
	fsm = FSM()
	fsm.main()