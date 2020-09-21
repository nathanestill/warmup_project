#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

class Vizualization(object):
	"""docstring for Vizualization"""
	def __init__(self):
		rospy.init_node("Vizualization")
		self.pub = rospy.Publisher("/vizMarks", Marker, queue_size=10)
	def main(self):
		while not rospy.is_shutdown():
			marker = Marker()
			marker.header.frame_id = "odom"
			marker.type = Marker.SPHERE
			marker.pose.position.x = 1
			marker.pose.position.y = 2
			marker.pose.position.z = 0
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.5;
			marker.scale.y = 0.5;
			marker.scale.z = 0.5;
			marker.color.a = 1.0
			marker.color.r = 1.0;
			marker.color.g = 0.5;
			marker.color.b = 0.0;
			self.pub.publish(marker)


if __name__ == '__main__':
	viz = Vizualization()
	viz.main()
		