#!/usr/bin/python

# A template to use for making a ROS node. Shows how to publish/subscribe to topics and how to structure the node as a class
# This is not the only way, but for consistency across our team try not to deviate too far so we can all have an easier time reading each other's code

# Import any necessary libraries
import rospy
import math
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from delta_robot.msg import DetectionArray
from delta_robot.msg import Detection
from delta_robot.msg import KFSPose
from delta_robot.msg import KFSPoseArray

class TaskPlanner(): 

	#Format of pose:
	#[x position cm, y position cm, z position cm, gripper angle rad, gripper open (positive)]

	def __init__(self): # This function is called one time as soon as an instance of a class is created
		self.rate = 3 #[Hz]

		# Declaring a subscriber. Parameters: topic name (string), topic data type, callback function
		# Every time another node publishes a message to "/arbitrary_subscribed_topic_name", we will call the function called self.arbitrary_topic_name_cb
		# The "/" before the topic name is important
		#rospy.Subscriber("/arbitrary_subscribed_topic_name", Float32MultiArray, self.get_state_cb)
		# Declaring a publisher. Parameters: topic name (string), topic data type, queue_size (use queue_size=10 as default)
		#self.delta_state_pub = rospy.Publisher("/arbitrary_published_topic_name", Float32MultiArray, queue_size=10)

		#Subscribers
		rospy.Subscriber("/finished_task",Bool,self.finished_trajectory_cb)
		#rospy.Subscriber("/mobile_arrived", Bool,self.mobile_arrived_cb)
		#rospy.Subscriber("/calibration_finished", Bool, queue_size=10, self.calibration_finished_cb)
		#rospy.Subscriber("/initialized", Bool, queue_size=10, self.odrive_initialized_cb)
		#rospy.Subscriber("/toppings", DetectionArray, self.detection_cb)
		#rospy.Subscriber("/slots", DetectionArray, self.slot_detection_cb)

		#publishers
		self.move_topping_pub = rospy.Publisher("/place_topping", KFSPoseArray, queue_size=10)
		self.move_to_pub = rospy.Publisher("/move_to", KFSPose, queue_size=10)
		#self.push_pizza_pub = rospy.Publisher("/push_pizza", KFSPose, queue_size=10)
		#self.shaker_pub = rospy.Publisher("/shake_salt", KFSPoseArray, queue_size=10)
		#self.press_dough_pub = rospy.Publisher("/press_dough", KFSPoseArray, queue_size=10)
		#self.calibration_pub = rospy.Publisher("/start_calibration", Bool, queue_size=10)
		#self.mobile_ready_pub = rospy.Publisher("/pizza_loaded", Bool, queue_size=10)

		self.calibrated = False
		self.toppings = None
		self.slots = None
		self.pizza_center = None


		# Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
		#rospy.Timer(rospy.Duration(1./self.rate), self.update_window)
	def finished_trajectory_cb(self,msg):
		print "Finished Task"

	#Table z=-774


	def run_test_task(self):
		pose1 = KFSPose()
		pose1.position.x = -150
		pose1.position.y = -150
		pose1.position.z = -770
		pose1.orientation = -0.5

		pose2 = KFSPose()
		pose2.position.x = 150
		pose2.position.y = 150
		pose2.position.z = -745
		pose2.orientation = 0.5

		move_msg = KFSPoseArray()
		move_msg.poses = []
		move_msg.poses.append(pose1)
		move_msg.poses.append(pose2)
		self.move_topping_pub.publish(move_msg)

	def run_test_task2(self):
		'''pose1 = KFSPose()
		pose1.position.x = 0
		pose1.position.y = 0
		pose1.position.z = -773
		pose1.orientation = 0
		pose1.open = True

		self.move_to_pub.publish(pose1)
		rospy.sleep(10)

		pose1 = KFSPose()
		pose1.position.x = 0
		pose1.position.y = 0
		pose1.position.z = -773
		pose1.orientation = 0
		pose1.open = False

		self.move_to_pub.publish(pose1)
		rospy.sleep(10)'''

		pose2 = KFSPose()
		pose2.position.x = 0
		pose2.position.y = 0
		pose2.position.z = -714
		pose2.orientation = 0
		pose2.open = False

		self.move_to_pub.publish(pose2)

if __name__ == "__main__":
	rospy.init_node('task_planner', anonymous=True) # Initialize the node
	taskplanner = TaskPlanner()
	rospy.sleep(1)
	taskplanner.run_test_task2()
	#rospy.spin() # Keeps python from exiting until the ROS node is stopped

