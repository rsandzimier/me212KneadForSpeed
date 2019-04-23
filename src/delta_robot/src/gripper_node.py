#!/usr/bin/python

# A template to use for making a ROS node. Shows how to publish/subscribe to topics and how to structure the node as a class
# This is not the only way, but for consistency across our team try not to deviate too far so we can all have an easier time reading each other's code

# Import any necessary libraries
import rospy
import math
from std_msgs.msg import Float32MultiArray # A standard message type that allows you to use topics that publish float arrays See http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html
# There are more standard messages (std_msgs.msg) available if needed. You can also create your own custom messages if needed.
# You can also import your own python files you write (can help organize code if it is long and you want to break it up into multiple files)

class Gripper: # Class name capitalized

	def __init__(self): # This function is called one time as soon as an instance of a class is created
		self.rate = 100 #[Hz]

		# Declaring a subscriber. Parameters: topic name (string), topic data type, callback function
		# Every time another node publishes a message to "/arbitrary_subscribed_topic_name", we will call the function called self.arbitrary_topic_name_cb
		# The "/" before the topic name is important
		#rospy.Subscriber("/arbitrary_subscribed_topic_name", Float32MultiArray, self.arbitrary_topic_name_cb)
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

if __name__ == "__main__":
	rospy.init_node('gripper_node', anonymous=True) # Initialize the node
	g = Gripper() # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 

	rospy.spin() # Keeps python from exiting until the ROS node is stopped

