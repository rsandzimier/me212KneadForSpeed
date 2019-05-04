#!/usr/bin/python


import rospy
import math
from std_msgs.msg import Float32MultiArray 
from nav_msgs.msg import Odometry

class Test: 
	def __init__(self): # This function is called one time as soon as an instance of a class is created
		self.rate = 100 #[Hz]

		self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
		self.at_odom_pub = rospy.Publisher("/apriltag_odom", Odometry, queue_size=10)


		rospy.Timer(rospy.Duration(1./self.rate), self.publish)

	def publish(self,event): 
		odom_msg = Odometry()
		at_odom_msg = Odometry()
		odom_msg.pose.covariance = (1,0,0,0,0,0,
									0,1,0,0,0,0,
									0,0,1,0,0,0,
									0,0,0,1,0,0,
									0,0,0,0,1,0,
									0,0,0,0,0,1)
		odom_msg.pose.pose.orientation.x = 1
		odom_msg.header.frame_id = "robot_base"
		odom_msg.header.stamp =rospy.get_rostime()
		at_odom_msg.pose.covariance = (1,0,0,0,0,0,
									0,1,0,0,0,0,
									0,0,1,0,0,0,
									0,0,0,1,0,0,
									0,0,0,0,1,0,
									0,0,0,0,0,1)
		at_odom_msg.pose.pose.orientation.x = 1
		at_odom_msg.header.frame_id = "robot_base"
		at_odom_msg.header.stamp = rospy.get_rostime()

		self.odom_pub.publish(odom_msg)
		self.at_odom_pub.publish(at_odom_msg)

if __name__ == "__main__":
	rospy.init_node('test', anonymous=True) # Initialize the node
	t = Test() # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 

	rospy.spin() # Keeps python from exiting until the ROS node is stopped

