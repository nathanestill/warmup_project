#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import ColorRGBA, Header

from visualization_msgs.msg import Marker

class Wallfollow(object):
	"""docstring for Wallfollow"""
	def __init__(self):
		rospy.init_node("Wallfollow")
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.sub = rospy.Subscriber("/scan", LaserScan, self.scanCB)
		self.rate = rospy.Rate(10)
		self.laserscan = None
		self.vis_pub = rospy.Publisher('Wall', Marker, queue_size=10)


	def visWall(self):
		markerPoints = []
		marker = Marker() # marks the path that the tractor will take
		marker.header.frame_id = "odom"
		marker.header.stamp = rospy.Time.now()
		marker.scale = Vector3(0.02, 0.02, 0.02)
		marker.color = ColorRGBA(1,1,0,1) # white
		marker.type = marker.POINTS
		for i in range(len(self.laserscan)):
			pointx = self.laserscan[i] * math.cos(math.radians(i))
			pointy = self.laserscan[i] * math.sin(math.radians(i))
			marker.points.append(Point(pointx,pointy,0.0))
		self.vis_pub.publish(marker);

	def scanCB(self,msg):
		self.laserscan = msg.ranges

	def followWall(self):
		averageAhead = 0
		averageBehind = 0
		for i in range(30):
			averageAhead += self.laserscan[i+30]
			averageBehind += self.laserscan[i+120]
		averageAhead /= 30
		averageBehind /= 30
		if(averageAhead > averageBehind):
			self.pub.publish(Twist(linear=Vector3(x=0.2),angular=Vector3(z=-0.2)))
		else:
			self.pub.publish(Twist(linear=Vector3(x=0.2),angular=Vector3(z=0.2)))

	def main(self):
		while((not rospy.is_shutdown()) and self.laserscan == None):
			rospy.loginfo("No data yet")
		while(not rospy.is_shutdown()):
			self.followWall()
			self.visWall()
			self.rate.sleep()
		self.pub.publish(Twist())

if __name__ == '__main__':
	wa = Wallfollow()
	wa.main()
		

