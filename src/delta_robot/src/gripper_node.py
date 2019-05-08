#!/usr/bin/python

# A template to use for making a ROS node. Shows how to publish/subscribe to topics and how to structure the node as a class
# This is not the only way, but for consistency across our team try not to deviate too far so we can all have an easier time reading each other's code

# Import any necessary libraries
import rospy
import math
import numpy
from delta_robot.msg import GripperPosition
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64 # A standard message type that allows you to use topics that publish float arrays See http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html
# There are more standard messages (std_msgs.msg) available if needed. You can also create your own custom messages if needed.
# You can also import your own python files you write (can help organize code if it is long and you want to break it up into multiple files)

class Gripper: # Class name capitalized

	# Do initialization of class variables
	SPOOL_OPEN = -1.6
	SPOOL_CLOSED = 0.4
	THRESHOLD = 0.1

	#Multiplier from the angle to the desired encoder reading, and the encoder offset for "zero" angle
	MOTOR_MULTIPLIER = 1 #4096/2/numpy.pi
	MOTOR_OFFSET = 0

	#self.gripperdata = None

	def __init__(self): # This function is called one time as soon as an instance of a class is created
		self.rate = 100 #[Hz]

		# Declaring a subscriber. Parameters: topic name (string), topic data type, callback function
		# Every time another node publishes a message to "/arbitrary_subscribed_topic_name", we will call the function called self.arbitrary_topic_name_cb
		# The "/" before the topic name is important
		rospy.Subscriber("/gripper_commands", GripperPosition, self.gripper_commands_cb)
		rospy.Subscriber("/spool_controller/state", JointState, self.update_spool_state_cb)
		rospy.Subscriber("/axis_controller/state", JointState, self.update_axis_state_cb)
		# Declaring a publisher. Parameters: topic name (string), topic data type, queue_size (use queue_size=10 as default)
		self.axis_controller_pub = rospy.Publisher("/axis_controller/command", Float64, queue_size=10)
		self.spool_controller_pub = rospy.Publisher("/spool_controller/command", Float64, queue_size=10)
		self.gripper_state_pub = rospy.Publisher("/gripper_state", GripperPosition, queue_size=10)

		self.gripperstate = [False,0]

		# Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
		rospy.Timer(rospy.Duration(1./self.rate), self.publish_state)

	def gripper_commands_cb(self,msg): # the _cb suffix stands for callback. Use this suffix on your callback functions for clarity
		# This function is called every time another node publishes to the topic called "/arbitrary_subscribed_topic_name"
		# msg is the message that was published to "/arbitrary_subscribed_topic_name"
		# Do whatever needs to happen when we receive this message. 
		# For example, set a class variable to the float array in the message and increment another class variable:
		

		if (msg.open == True):# and self.gripperstate[0] == False):
			self.open_gripper()
		elif (msg.open == False):# and self.gripperstate[0] == True):
			self.close_gripper()
		#self.gripperstate[0] = msg.open

		self.set_gripper_angle(msg.angle)
		#self.gripperstate[1] = msg.angle



		#self.arbitrary_class_variable_name2 = self.arbitrary_class_variable_name2 + 1

	def publish_state(self, event):
		#publishes to gripper_state topic
		statemsg = GripperPosition()
		statemsg.open = self.gripperstate[0]
		statemsg.angle = self.gripperstate[1]
		self.gripper_state_pub.publish(statemsg)

	def update_axis_state_cb(self, msg):
		#updates the axis motor state
		self.gripperstate[1] = msg.current_pos

	def update_spool_state_cb(self, msg):
		#updates the spool motor state
		if (self.gripperstate[0] == True):
			if (msg.current_pos > self.SPOOL_CLOSED - self.THRESHOLD):
				self.gripperstate[0] = False
		elif (self.gripperstate[0] == False):
			if (msg.current_pos < self.SPOOL_OPEN + self.THRESHOLD):
				self.gripperstate[0] = True


	def open_gripper(self):
		#Send the command to open the gripper
		self.spool_controller_pub.publish(Float64(self.SPOOL_OPEN*self.MOTOR_MULTIPLIER+self.MOTOR_OFFSET))
		rospy.loginfo("opened gripper")

	def close_gripper(self):
		#Send the command to close the gripper
		self.spool_controller_pub.publish(Float64(self.SPOOL_CLOSED*self.MOTOR_MULTIPLIER+self.MOTOR_OFFSET))
		rospy.loginfo("closed gripper")

	def set_gripper_angle(self,desired_angle):
		#Send the command to rotate the gripper to the desired angle
		#desired_angle = numpy.clip(1.1,-1.1,desired_angle)
		encoder = desired_angle*self.MOTOR_MULTIPLIER + self.MOTOR_OFFSET

		self.axis_controller_pub.publish(Float64(encoder))
		rospy.loginfo("set gripper angle to "+str(desired_angle)+" at encoder setting "+str(encoder))


if __name__ == "__main__":
	rospy.init_node('gripper_node', anonymous=True) # Initialize the node
	g = Gripper() # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 

	rospy.spin() # Keeps python from exiting until the ROS node is stopped

