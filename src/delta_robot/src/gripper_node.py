#!/usr/bin/python

# A template to use for making a ROS node. Shows how to publish/subscribe to topics and how to structure the node as a class
# This is not the only way, but for consistency across our team try not to deviate too far so we can all have an easier time reading each other's code

# Import any necessary libraries
import rospy
import math
import numpy
from std_msgs.msg import Float64 # A standard message type that allows you to use topics that publish float arrays See http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html
# There are more standard messages (std_msgs.msg) available if needed. You can also create your own custom messages if needed.
# You can also import your own python files you write (can help organize code if it is long and you want to break it up into multiple files)

class Gripper: # Class name capitalized

	def __init__(self): # This function is called one time as soon as an instance of a class is created
		self.rate = 10 #[Hz]

		# Declaring a subscriber. Parameters: topic name (string), topic data type, callback function
		# Every time another node publishes a message to "/arbitrary_subscribed_topic_name", we will call the function called self.arbitrary_topic_name_cb
		# The "/" before the topic name is important
		#rospy.Subscriber("/arbitrary_subscribed_topic_name", Float32MultiArray, self.arbitrary_topic_name_cb)
		# Declaring a publisher. Parameters: topic name (string), topic data type, queue_size (use queue_size=10 as default)
		self.axis_controller_pub = rospy.Publisher("/axis_controller/command", Float64, queue_size=10)
		self.spool_controller_pub = rospy.Publisher("/spool_controller/command", Float64, queue_size=10)

		# Do any other initialization of class
		self.spool_open = 2000
		self.spool_closed = 1000
		self.gripperstate = [0,0]

		#Multiplier from the angle to the desired encoder reading, and the encoder offset for "zero" angle
		self.motor_multiplier = 30
		self.motor_offset = 1000

		self.gripperdata = None


		# Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
		rospy.Timer(rospy.Duration(1./self.rate), self.function_to_loop)

	def gripper_actuate_cb(self,msg): # the _cb suffix stands for callback. Use this suffix on your callback functions for clarity
		# This function is called every time another node publishes to the topic called "/arbitrary_subscribed_topic_name"
		# msg is the message that was published to "/arbitrary_subscribed_topic_name"
		# Do whatever needs to happen when we receive this message. 
		# For example, set a class variable to the float array in the message and increment another class variable:
		gripperdata = msg.data

		if (gripperdata[0] == 1 and self.gripperstate[0] == 0):
			self.open_gripper()
		else if (gripperdata[0] == 0 and self.gripperstate[0] == 1):
			self.close_gripper()
		self.gripperstate[0] = gripperdata[0]

		self.set_gripper_angle(gripperdata[1])
		self.gripperstate[1] = gripperdata[1]



		#self.arbitrary_class_variable_name2 = self.arbitrary_class_variable_name2 + 1

	def open_gripper(self):
		#Send the command to open the gripper
		self.spool_controller.publish(std_msgs.msg.Float64(self.spool_open))
		rospy.loginfo("opened gripper")

	def close_gripper(self):
		#Send the command to close the gripper
		self.spool_controller.publish(std_msgs.msg.Float64(self.spool_closed))
		rospy.loginfo("closed gripper")

	def set_gripper_angle(self,desired_angle)
		#Send the command to rotate the gripper to the desired angle
		desired_angle = numpy.clip(-70,70,desired_angle)
		encoder = desired_angle*self.motor_multiplier + self.motor_offset

		self.axis_controller.publish(std_msgs.msg.Float64(encoder))
		rospy.loginfo("set gripper angle to "+str(desired_angle)+" at encoder setting "+str(encoder))


if __name__ == "__main__":
	rospy.init_node('gripper_node', anonymous=True) # Initialize the node
	g = Gripper() # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 

	rospy.spin() # Keeps python from exiting until the ROS node is stopped

