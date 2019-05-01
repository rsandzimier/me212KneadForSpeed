#!/usr/bin/python

import rospy
import math
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Bool 
from delta_robot.msg import KFSPoseArray
from delta_robot.msg import KFSPose

class TrajectoryPlanner: 
	def __init__(self): 
		self.rate = 100 #[Hz]
		
		rospy.Subscriber("/place_topping", KFSPoseArray, self.place_topping_cb)
		rospy.Subscriber("/shake_salt", KFSPoseArray, self.shake_salt_cb)
		rospy.Subscriber("/press_dough", KFSPoseArray, self.press_dough_cb)
		rospy.Subscriber("/push_pizza", KFSPose, self.push_pizza_cb)
		rospy.Subscriber("/move_to", KFSPose, self.move_to_cb)

		self.finished_task_pub = rospy.Publisher("/finished_task", Bool, queue_size=10)

		rospy.Timer(rospy.Duration(1./self.rate), self.function_to_loop)

	def place_topping_cb(self,msg): 
		pass
	def shake_salt_cb(self,msg): 
		pass
	def press_dough_cb(self,msg): 
		pass
	def push_pizza_cb(self,msg): 
		pass
	def move_to_cb(self,msg): 
		pass


if __name__ == "__main__":
	rospy.init_node('trajectory_planner', anonymous=True) # Initialize the node
	tp = TrajectoryPlanner() # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 

	rospy.spin() # Keeps python from exiting until the ROS node is stopped

