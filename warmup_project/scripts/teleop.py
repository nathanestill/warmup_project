#!/usr/bin/env python3
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3

class Teleop(object):
	"""docstring for Teleop"""
	def __init__(self):
		rospy.init_node("Teleop")
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.settings = termios.tcgetattr(sys.stdin)
		self.key = None

	def getKey(self):
	    tty.setraw(sys.stdin.fileno())
	    select.select([sys.stdin], [], [], 0)
	    self.key = sys.stdin.read(1)
	    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

	def main(self):
		while self.key != '\x03':
			self.getKey()
			if(self.key == "u"):
				self.pub.publish(Twist(linear=Vector3(x=1.0),angular=Vector3(z=1.0)))
			elif(self.key == "i"):
				self.pub.publish(Twist(linear=Vector3(x=1.0),angular=Vector3(z=0.0)))
			elif(self.key == "o"):
				self.pub.publish(Twist(linear=Vector3(x=1.0),angular=Vector3(z=-1.0)))
			elif(self.key == "j"):
				self.pub.publish(Twist(linear=Vector3(x=0.0),angular=Vector3(z=1.0)))
			elif(self.key == "k"):
				self.pub.publish(Twist(linear=Vector3(x=0.0),angular=Vector3(z=0.0)))
			elif(self.key == "l"):
				self.pub.publish(Twist(linear=Vector3(x=0.0),angular=Vector3(z=-1.0)))
			elif(self.key == "m"):
				self.pub.publish(Twist(linear=Vector3(x=-1.0),angular=Vector3(z=1.0)))
			elif(self.key == ","):
				self.pub.publish(Twist(linear=Vector3(x=-1.0),angular=Vector3(z=0.0)))
			elif(self.key == "."):
				self.pub.publish(Twist(linear=Vector3(x=-1.0),angular=Vector3(z=-1.0)))

			rospy.loginfo("Key Pressed: " + self.key)

if __name__ == '__main__':
	te = Teleop()
	te.main()
