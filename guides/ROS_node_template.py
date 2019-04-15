#!/usr/bin/python

# A template to use for making a ROS node. Shows how to publish/subscribe to topics and how to structure the node as a class
# This is not the only way, but for consistency across our team try not to deviate too far so we can all have an easier time reading each other's code

# Import any necessary libraries
import rospy
import math
from std_msgs.msg import Float32MultiArray # A standard message type that allows you to use topics that publish float arrays See http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html
# There are more standard messages (std_msgs.msg) available if needed. You can also create your own custom messages if needed.
# You can also import your own python files you write (can help organize code if it is long and you want to break it up into multiple files)

class ArbitraryClassName: # Class name capitalized
	# Define any constant variables for for class. To use these variables in class functions, use self.ARBITRARY_CONSTANT_INT_NAME, etc
	# Use all caps for these constants
	ARBITRARY_CONSTANT_INT_NAME = 1000 
	ARBITRARY_CONSTANT_STRING_NAME = "Hello World"

	def __init__(self): # This function is called one time as soon as an instance of a class is created
		self.rate = 100 #[Hz]

		# Declaring a subscriber. Parameters: topic name (string), topic data type, callback function
		# Every time another node publishes a message to "/arbitrary_subscribed_topic_name", we will call the function called self.arbitrary_topic_name_cb
		# The "/" before the topic name is important
		rospy.Subscriber("/arbitrary_subscribed_topic_name", Float32MultiArray, self.arbitrary_topic_name_cb)
		# Declaring a publisher. Parameters: topic name (string), topic data type, queue_size (use queue_size=10 as default)
		self.arbitrary_topic_name_pub = rospy.Publisher("/arbitrary_published_topic_name", Float32MultiArray, queue_size=10)

		# Do any other initialization of class
		self.arbitrary_class_variable_name1 = []
		self.arbitrary_class_variable_name2 = 0

		# Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
		rospy.Timer(rospy.Duration(1./self.rate), self.function_to_loop)

	def arbitrary_topic_name_cb(self,msg): # the _cb suffix stands for callback. Use this suffix on your callback functions for clarity
		# This function is called every time another node publishes to the topic called "/arbitrary_subscribed_topic_name"
		# msg is the message that was published to "/arbitrary_subscribed_topic_name"
		# Do whatever needs to happen when we receive this message. 
		# For example, set a class variable to the float array in the message and increment another class variable:
		self.arbitrary_class_variable_name1 = msg.data
		self.arbitrary_class_variable_name2 = self.arbitrary_class_variable_name2 + 1

	def function_to_loop(self,event): # This is the function that we set a timer for above. 
		# Do whatever you would like to do at the rate specified by the timer. 
		# For example: print some stuff out, and publish to a topic if a certain criteria is met
		print self.ARBITRARY_GLOBAL_STRING_NAME

		if self.arbitrary_class_variable_name2 > self.ARBITRARY_GLOBAL_INT_NAME:
			arbitrary_message_name_msg = Float32MultiArray() # Empty message of type Float32MultiArray
			arbitrary_message_name_msg.data = self.arbitrary_class_variable_name2 # Populate the message
			self.arbitrary_topic_name_pub.publish(arbitrary_message_name_msg) # Publish the message to the topic "/arbitrary_published_topic_name"
		else:
			print self.arbitrary_function_name() # Call some function and print out whatever it returns

	def arbitrary_function_name(self): # Meaningless function just as an example
		return math.pi*self.ARBITRARY_GLOBAL_INT_NAME 

if __name__ == "__main__":
	rospy.init_node('arbitrary_node_name', anonymous=True) # Initialize the node
	acn = ArbitraryClassName() # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 

	rospy.spin() # Keeps python from exiting until the ROS node is stopped

