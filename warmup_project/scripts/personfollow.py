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

class Personfollow(object):
	"""docstring for Personfollow"""
	def __init__(self):
		rospy.init_node("Personfollow")
		self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
		self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCB)
		self.vis_pub = rospy.Publisher("/vis_scan",Marker,queue_size=10)
		self.visPersonPub = rospy.Publisher("/person",Marker,queue_size=10)
		self.laser = None
		self.Person = Point()
		self.rate = rospy.Rate(10)

	def laserCB(self,msg):
		self.laser = msg.ranges

	def detectPerson(self):
		closest = 0
		for i in range(len(self.laser)):
			if(self.laser[i] < self.laser[closest]):
				closest = i
		rospy.loginfo(self.laser[closest])
		self.Person.x = self.laser[closest] * math.cos(math.radians(closest))
		self.Person.y = self.laser[closest] * math.sin(math.radians(closest))

	def goToPerson(self):
		if(abs(self.Person.x) < 0.3 and abs(self.Person.y) < 0.1):
			self.pub.publish(Twist())
		else:
			self.pub.publish(Twist(linear=Vector3(x=numpy.clip(self.Person.x,-1.5,1.5)),angular=Vector3(z=numpy.clip(self.Person.y,-1.5,1.5))))

	def visWall(self):
		marker = Marker() # marks the path that the tractor will take
		marker.header.frame_id = "odom"
		marker.header.stamp = rospy.Time.now()
		marker.scale = Vector3(0.02, 0.02, 0.02)
		marker.color = ColorRGBA(1,1,0,1) # white
		marker.type = marker.POINTS
		for i in range(len(self.laser)):
			if(self.laser[i] < 20):
				pointx = self.laser[i] * math.cos(math.radians(i))
				pointy = self.laser[i] * math.sin(math.radians(i))
				marker.points.append(Point(pointx,pointy,0.0))
		self.vis_pub.publish(marker);


		personMarker = Marker()
		personMarker.header.frame_id = "odom"
		personMarker.header.stamp = rospy.Time.now()
		personMarker.scale = Vector3(0.1, 0.1, 0.1)
		personMarker.color = ColorRGBA(0,0,1,1) # white
		personMarker.type = personMarker.SPHERE
		personMarker.pose.position.x = self.Person.x;
		personMarker.pose.position.y = self.Person.y;
		personMarker.pose.position.z = self.Person.z;
		personMarker.pose.orientation.x = 0.0;
		personMarker.pose.orientation.y = 0.0;
		personMarker.pose.orientation.z = 0.0;
		personMarker.pose.orientation.w = 1.0;
		self.visPersonPub.publish(personMarker)

	def main(self):
		while((not rospy.is_shutdown()) and self.laser == None):
			rospy.loginfo("No data yet")
		while(not rospy.is_shutdown()):
			self.detectPerson()
			self.goToPerson()
			self.visWall()
			self.rate.sleep()


if __name__ == '__main__':
	PF = Personfollow()
	PF.main()
