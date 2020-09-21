#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math

class Square(object):
	"""docstring for Square"""
	def __init__(self):
		rospy.init_node("Square")
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.odomSub = rospy.Subscriber("/odom", Odometry, self.odomCB)
		self.rate = rospy.Rate(10)
		self.state = 0
		self.position = None
		self.orientation = None

	def odomCB(self,msg):
		self.position = msg.pose.pose.position
		self.orientation = euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0]

	def goStraight(self, dist):
		if(self.distanceBetweenTwoPositions(self.registeredPos,self.position) > dist):
			self.state = 1
			self.pub.publish(Twist())
			self.registeredOrient = self.orientation
		else:
			self.pub.publish(Twist(linear=Vector3(x=0.3)))

	def turnRight(self):
		rospy.loginfo(str(self.registeredOrient) + " " + str(self.orientation))
		if(self.registeredOrient+0.1 < self.orientation):	
			self.registeredOrient += 2*math.pi
		if(self.orientation + math.pi/2 <= self.registeredOrient):
			self.state = 0
			self.pub.publish(Twist())
			self.registeredPos = self.position
		else:
			self.pub.publish(Twist(angular=Vector3(z=0.3)))
	def distanceBetweenTwoPositions(self,p1,p2):
		return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)

	def main(self):
		while((self.position == None or self.orientation == None) and not rospy.is_shutdown()):
			rospy.loginfo("No odom data yet")
		self.registeredPos = self.position
		while not rospy.is_shutdown():
			if(self.state == 0):
				self.goStraight(1)
			else:
				self.turnRight()
			self.rate.sleep

if __name__ == '__main__':
	sq = Square()
	sq.main()
