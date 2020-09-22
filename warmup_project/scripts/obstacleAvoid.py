#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import ColorRGBA, Header
import numpy

from visualization_msgs.msg import Marker

class ObstacleAvoid(object):
	"""docstring for Personfollow"""
	def __init__(self):
		rospy.init_node("ObstacleAvoid")
		self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
		self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCB)
		self.vis_pub = rospy.Publisher("/vis_scan",Marker,queue_size=10)
		self.visPersonPub = rospy.Publisher("/person",Marker,queue_size=10)
		self.odomSub = rospy.Subscriber("/odom", Odometry, self.odomCB)
		self.laser = None
		self.position = None
		self.orientation = None
		self.Person = Point()
		self.rate = rospy.Rate(10)

	def laserCB(self,msg):
		self.laser = msg.ranges

	def odomCB(self,msg):
		self.position = msg.pose.pose.position
		self.orientation = euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0]

	def findObstacles(self):
		self.obstacleVectors = Point()
		for i in range(len(self.laser)):
			if(self.laser[i] < 5):
				self.obstacleVectors.x -= (0.01 / self.laser[i]) * math.cos(math.radians(i))
				self.obstacleVectors.y -= (0.01 / self.laser[i]) * math.sin(math.radians(i))

	def goToGoal(self):
		goalPoint = Point(x=10.0,y=0.0)
		x = -math.cos(self.orientation) * (goalPoint.x - self.position.x) + math.sin(self.orientation) * (goalPoint.y - self.position.y) 
		y = math.sin(self.orientation) * (self.position.x - goalPoint.x) + math.cos(self.orientation) * (goalPoint.y - self.position.y)
		self.goalVector = Point(x=x / numpy.linalg.norm([x,y]),y=y / numpy.linalg.norm([x,y]))

	def routeRobot(self):
		velCmd = Twist()
		velCmd.linear.x = self.obstacleVectors.x + self.goalVector.x
		velCmd.angular.z = self.obstacleVectors.y + self.goalVector.y
		self.pub.publish(velCmd)

	def main(self):
		while((not rospy.is_shutdown()) and (self.laser == None or self.position == None or self.orientation == None)):
			rospy.loginfo("No data yet")
		while(not rospy.is_shutdown()):
			self.findObstacles()
			self.goToGoal()
			self.routeRobot()
			self.rate.sleep()

if __name__ == '__main__':
	obs = ObstacleAvoid()
	obs.main()