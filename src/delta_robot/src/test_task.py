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

class TestTask(): 

	DEBUG = False

	#Format of pose:
	#[x position cm, y position cm, z position cm, gripper angle rad, gripper open (positive)]

	def __init__(self): # This function is called one time as soon as an instance of a class is created
		self.rate = 20 #[Hz]

		# Declaring a subscriber. Parameters: topic name (string), topic data type, callback function
		# Every time another node publishes a message to "/arbitrary_subscribed_topic_name", we will call the function called self.arbitrary_topic_name_cb
		# The "/" before the topic name is important
		#rospy.Subscriber("/arbitrary_subscribed_topic_name", Float32MultiArray, self.get_state_cb)
		# Declaring a publisher. Parameters: topic name (string), topic data type, queue_size (use queue_size=10 as default)
		#self.delta_state_pub = rospy.Publisher("/arbitrary_published_topic_name", Float32MultiArray, queue_size=10)

		#Subscribers
		rospy.Subscriber("/finished_task",Bool,self.finished_task_cb)
		#rospy.Subscriber("/mobile_arrived", Bool,self.mobile_arrived_cb)
		#rospy.Subscriber("/calibration_finished", Bool, queue_size=10, self.calibration_finished_cb)
		#rospy.Subscriber("/initialized", Bool, queue_size=10, self.odrive_initialized_cb)
		#rospy.Subscriber("/toppings", DetectionArray, self.detection_cb)
		#rospy.Subscriber("/slots", DetectionArray, self.slot_detection_cb)

		#publishers
		self.move_topping_pub = rospy.Publisher("/place_topping", KFSPoseArray, queue_size=10)
		self.move_to_pub = rospy.Publisher("/move_to", KFSPose, queue_size=10)
		self.push_pizza_pub = rospy.Publisher("/push_pizza", KFSPose, queue_size=10)
		self.shaker_pub = rospy.Publisher("/shake_salt", KFSPoseArray, queue_size=10)
		self.press_dough_pub = rospy.Publisher("/press_dough", KFSPoseArray, queue_size=10)
		#self.calibration_pub = rospy.Publisher("/start_calibration", Bool, queue_size=10)
		#self.mobile_ready_pub = rospy.Publisher("/pizza_loaded", Bool, queue_size=10)
		self.toppings_pub = rospy.Publisher("/toppings", DetectionArray, queue_size=10)
		self.slots_pub = rospy.Publisher("/slots", DetectionArray, queue_size=10)

		self.traj_fin_pub = rospy.Publisher("/finished_trajectory", Bool, queue_size=10)
		self.rt_pub = rospy.Publisher("/run_task", Bool, queue_size=10)
		self.ma_pub = rospy.Publisher("/mobile_arrived", Bool, queue_size=10)
		self.pl_pub = rospy.Publisher("/pizza_loaded", Bool, queue_size=10)

		self.calibrated = False
		self.toppings = None
		self.slots = None
		self.pizza_center = None

		self.x = -150.0 #variable to move the objects slightly for average testing


		# Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
		rospy.Timer(rospy.Duration(1./self.rate), self.feed_slot_detections)
	
	def finished_task_cb(self,msg):
		print "Finished Task"

	#Table z=-774
	def run_full_test(self):
		print("Publishing locations repeatedly:")
		for i in range(50):
			self.run_test_task_3(None)
			rospy.sleep(0.2)
		print("Finished publishing repeat locations")
		self.rt_pub.publish(True)
		print("Enter 0 to fire a trajectory finished message, 1 to exit loop")
		i = input()
		while (i != 1):
			print("sent finished message")
			self.traj_fin_pub.publish(True)
			i = input()
		print("waiting to send MR arrived message (5 sec)")
		rospy.sleep(5)
		print("sending MR arrived message")
		self.ma_pub.publish(True)
		print("waiting to send pizza loaded message (2 sec)")
		rospy.sleep(2)
		print("sending pizza loaded message")
		self.pl_pub.publish(True)
		print("No more messages to send")

	def test_traj_planner(self):
		print("Testing move topping")
		pose1 = KFSPose()
		pose1.position.x = -150
		pose1.position.y = -150
		pose1.position.z = -769
		pose1.orientation = -0.0

		pose2 = KFSPose()
		pose2.position.x = 25
		pose2.position.y = -15
		pose2.position.z = -750
		pose2.orientation = 0.0

		move_msg = KFSPoseArray()
		move_msg.poses = []
		move_msg.poses.append(pose1)
		move_msg.poses.append(pose2)
		self.move_topping_pub.publish(move_msg)

		input("Press any key to continue...")

		print("Testing press dough")
		pose1.position.x = 150
		pose1.position.y = -150
		pose1.position.z = -767
		pose1.orientation = -0.0

		pose2.position.x = 20
		pose2.position.y = -220
		pose2.position.z = -762
		pose2.orientation = 0.0

		move_msg = KFSPoseArray()
		move_msg.poses = []
		move_msg.poses.append(pose1)
		move_msg.poses.append(pose2)
		self.press_dough_pub.publish(move_msg)
		input("Press any key to continue")

		print("testing push pizza")
		pose2 = KFSPose()
		pose2.position.x = 0
		pose2.position.y = 0
		pose2.position.z = -760
		pose2.orientation = 0.0
		self.push_pizza_pub.publish(pose2)
		input("press any key to continue")

		print("testing shake salt")
		pose1 = KFSPose()
		pose1.position.x = -150
		pose1.position.y = 150
		pose1.position.z = -767
		pose1.orientation = 0.0

		pose2.position.x = 0
		pose2.position.y = 0
		pose2.position.z = -720
		pose2.orientation = 0.0

		move_msg = KFSPoseArray()
		move_msg.poses = []
		move_msg.poses.append(pose1)
		move_msg.poses.append(pose2)
		self.shaker_pub.publish(move_msg)
		input("press any key to continue")

		print("move to center")
		pose2.position.x = 0
		pose2.position.y = 0
		pose2.position.z = -714.66
		pose2.orientation = 0.0
		pose2.open = False
		self.move_to_pub.publish(pose2)
		print("done!")


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

	def create_detection(self,x,y,z,a,t):
		p = Point()
		p.x = x
		p.y = y
		p.z = z
		d = Detection()
		d.position = p
		d.orientation = a
		d.type = t
		return d

	def run_test_task_3(self, extra):
		#Create some detections for testing
		d = [None,None,None,None,None,None,None,None,None,None,None,None,None,None,None]
		d[0] = self.create_detection(self.x, self.x, -770.0, -0.5, 1)
		d[1] = self.create_detection(-140.0, -150.0, -770.0, -0.3, 2)
		d[2] = self.create_detection(-120.0, -150.0, -770.0, -0.1, 2)
		d[3] = self.create_detection(-110.0, -150.0, -770.0, -0.4, 1)
		d[4] = self.create_detection(-100.0, -150.0, -770.0, -0.4, 1)
		d[5] = self.create_detection(-150.0, -140.0, -770.0, -0.5, 3)
		d[6] = self.create_detection(-150.0, -130.0, -770.0, -0.5, 0)
		d[7] = self.create_detection(-150.0, -120.0, -770.0, -0.3, 4)
		d[8] = self.create_detection(-150.0, -110.0, -770.0, -0.5, 4)
		d[9] = self.create_detection(-150.0, -100.0, -770.0, -0.1, 2)
		d[10] = self.create_detection(-140.0, -140.0, -770.0, -0.0, 1)
		d[11] = self.create_detection(-130.0, -130.0, -770.0, -0.0, 1)
		d[12] = self.create_detection(-120.0, -120.0, -770.0, -0.1, 2)
		d[13] = self.create_detection(-110.0, -110.0, -770.0, -0.2, 0)
		d[14] = self.create_detection(-100, -100.0, -770.0, -0.4, 3)
		#self.x = self.x + 0.1
		da = DetectionArray()
		da.detections = d

		s = [None, None, None]
		s[0] = self.create_detection(150,140,-776,-0.5, 0)
		s[1] = self.create_detection(150,110,-776,-0.5, 0)
		s[2] = self.create_detection(-150,100,-776,-0.5, 0)

		sa = DetectionArray()
		sa.detections = s

		self.toppings_pub.publish(da)
		self.slots_pub.publish(sa)
		print("Published test topping and slot positions")

	def feed_slot_detections(self, extra):
		s = [None,None,None, None, None, None]
		s[0] = self.create_detection(150,140,-776,0.0, -1)
		s[1] = self.create_detection(150,100,-776,0.0, -1)
		s[2] = self.create_detection(-150,100,-776,0.0, -1)
		s[3] = self.create_detection(100,140,-776,0.0, -1)
		s[4] = self.create_detection(110,100,-776,0.0, -1)
		s[5] = self.create_detection(-120,-100,-776,0.0, -1)
		sa = DetectionArray()
		sa.detections = s
		self.slots_pub.publish(sa)


if __name__ == "__main__":
	rospy.init_node('task_planner', anonymous=True) # Initialize the node
	task = TestTask()
	#task.run_full_test()
	#rospy.sleep(1)
	#taskplanner.run_test_task2()
	#task.test_traj_planner()
	rospy.spin() # Keeps python from exiting until the ROS node is stopped

