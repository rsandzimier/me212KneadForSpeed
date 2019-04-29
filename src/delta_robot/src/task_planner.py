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
		rospy.Subscriber("/finished_trajectory",Bool,queue_size=10,self.finished_trajectory_cb)
		rospy.Subscriber("/mobile_arrived", Bool, queue_size=10,self.mobile_arrived_cb)
		#rospy.Subscriber("/calibration_finished", Bool, queue_size=10, self.calibration_finished_cb)
		#rospy.Subscriber("/initialized", Bool, queue_size=10, self.odrive_initialized_cb)
		rospy.Subscriber("/toppings", DetectionArray, queue_size=10, self.detection_cb)
		rospy.Subscriber("/slots", DetectionArray, queue_size=10, self.slot_detection_cb)

		#publishers
		self.move_topping_pub = rospy.Publisher("/move_topping", KFSPoseArray, queue_size=10)
		self.push_pizza_pub = rospy.Publisher("/push_pizza", KFSPose, queue_size=10)
		self.shaker_pub = rospy.Publisher("/shaker", KFSPoseArray, queue_size=10)
		#self.calibration_pub = rospy.Publisher("/start_calibration", Bool, queue_size=10)
		self.mobile_ready_pub = rospy.Publisher("/pizza_loaded", Bool, queue_size=10)

		self.calibrated = False
		self.toppings = None
		self.slots = None
		self.pizza_center = None


		# Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
		#rospy.Timer(rospy.Duration(1./self.rate), self.update_window)

	def bootstrap(self)
		if (calibrated == False): #robot not started yet
			rospy.Publisher("/initialize_motors", Bool, queue_size=10).publish(Bool())
			rospy.wait_for_message("/initialized", Bool)
			print("ODrives initialized")
			rospy.Publisher("/start_calibration", Bool, queue_size=10).publish(Bool())
			rospy.wait_for_message("/calibration_finished", Bool)
			print("Odrives calibrated")
		process_camera_output()

	def process_camera_output():
		#Reads in the data from the camera and uses an average of object positions
		pass

	def run_task(self):
		#Runs the entire delta robot task list

		#Step 1: Move toppings to pizza
		num_toppings = [2,2,2,2,1]
		#0 -- pepperoni
		#1 -- olive
		#2 -- ham
		#3 -- pineapple
		#4 -- anchovy
		#get toppings
		for slot in self.slots:
			topping = self.toppings.pop()
			if (topping == None):
				print("Out of toppings!")
				break
			#get type
			while (num_toppings[topping.type.get()] == 0):
				topping = self.toppings.pop() #keep looking for toppings until we get one that we can put on
			print("found topping at "+to_string(topping)+", placing in slot "+to_string(slot))

			move_msg = KFSPoseArray()
			slot_pose = KFSPose()
			topping_pose = KFSPose()
			slot_pose.position = slot.position
			slot_pose.orientation = 0
			topping_pose.position = topping.position
			topping_pose.orientation = topping.orientation
			move_msg.poses = [topping_pose, slot_pose]

			self.move_topping_pub.publish(move_msg)
			if (rospy.wait_for_message("/finished_trajectory", Bool).get() == True):
				print("Success!")
			else:
				print("Failed (no change in logic")



		#Step 2: Wait for MR to come to position, then move pizza to MR
		while (self.mobile_arrived == False):
			print("No MR, waiting...")
			rospy.sleep(1)

		self.push_pizza_pub.publish(self.pizza_center)
		if (rospy.wait_for_message("/pizza_loaded", Bool).get() == True):
			print("MR loaded!")
			self.mobile_ready_pub.publish(True)
		else:
			print("Failed to load MR")
			self.mobile_ready_pub.publish(True) #send it off anyway
		print("Done task!")

	def to_string(self, detection):
		return str(detection.position.x)+","+str(detection.position.y)+","+str(detection.position.z)+","+str(detection.orientation)+","+str(detection.type)

	#def get_delta_pos_cb(self,msg): # the _cb suffix stands for callback. Use this suffix on your callback functions for clarity
		#set the delta_current_pos with the content of the message

	#def choose_item(self,itemposlist):
	#	#Choose the closest item to the current delta_robot position
	#	dists = []
	#	i=0
	#	for itempos in itemposlist:
	#		dists[i] = sqrdist(itempos, self.delta_current_pos)
	#		i = i + 1
	#	i = index(min(dists)) #gets the item index of the item with minimum distance to the robot
	#	return itemposlist[i]

	def sqrdist(self, pos1, pos2):
		#Returns the squared distance between 2 points
		return (pos1.x-pos2.x)**2 + (pos1.y-pos2.y)**2 + (pos1.z-pos2.z)**2

if __name__ == "__main__":
	rospy.init_node('task_planner', anonymous=True) # Initialize the node
	#rospy.spin() # Keeps python from exiting until the ROS node is stopped

