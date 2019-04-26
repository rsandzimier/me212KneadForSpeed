#!/usr/bin/python

# A template to use for making a ROS node. Shows how to publish/subscribe to topics and how to structure the node as a class
# This is not the only way, but for consistency across our team try not to deviate too far so we can all have an easier time reading each other's code

# Import any necessary libraries
import rospy
import math
#from std_msgs.msg import Float32MultiArray # A standard message type that allows you to use topics that publish float arrays See http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html
# There are more standard messages (std_msgs.msg) available if needed. You can also create your own custom messages if needed.
# You can also import your own python files you write (can help organize code if it is long and you want to break it up into multiple files)

class TaskPlanner(): 

	#Format of positions:
	#[x position cm, y position cm, z position cm, gripper angle rad]

	def __init__(self): # This function is called one time as soon as an instance of a class is created
		self.rate = 3 #[Hz]

		# Declaring a subscriber. Parameters: topic name (string), topic data type, callback function
		# Every time another node publishes a message to "/arbitrary_subscribed_topic_name", we will call the function called self.arbitrary_topic_name_cb
		# The "/" before the topic name is important
		#rospy.Subscriber("/arbitrary_subscribed_topic_name", Float32MultiArray, self.get_state_cb)
		# Declaring a publisher. Parameters: topic name (string), topic data type, queue_size (use queue_size=10 as default)
		#self.delta_state_pub = rospy.Publisher("/arbitrary_published_topic_name", Float32MultiArray, queue_size=10)

		self.delta_current_pos = [0,0,0,0]
		self.zrise = 10 #Amount for robot to rise above the pickup/dropoff location in cm

		#rospy.Subscriber("/finished_trajectory", Bool, )
		#pack everything
		#self.prompt.grid(side="top", fill="x");
		#self.entry.pack(side="top", fill="x", padx=20)
		#self.output.pack(side="top", fill="x", expand=True)
		#self.submit.pack(side="right")

		# Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
		rospy.Timer(rospy.Duration(1./self.rate), self.update_window)

	def run_task(self):
		#Runs the entire delta robot task list

		#Step 1: Move toppings to pizza
		pepperoni = 2
		ham = 2
		olive = 2
		anchovy = 1
		pineapple = 2




		#Step 2: Wait for MR to come to position, then move pizza to MR

	def publish_state(self):
		#publishes the user defined coordinates to the delta robot

	def get_delta_pos_cb(self,msg): # the _cb suffix stands for callback. Use this suffix on your callback functions for clarity
		#set the delta_current_pos with the content of the message

	def choose_item(self,itemposlist):
		#Choose the closest item to the current delta_robot position
		dists = []
		i=0
		for itempos in itemposlist:
			dists[i] = sqrdist(itempos, self.delta_current_pos)
			i = i + 1
		i = index(min(dists)) #gets the item index of the item with minimum distance to the robot
		return itemposlist[i]

	def sqrdist(self, pos1, pos2):
		#Returns the squared distance between 2 points
		return (pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2

	def move_item(self,pos1,pos2):
		#Moves an item from pos1 to pos2, regardless of whether there is actually an item at pos1
		abovepos1 = pos1.copy()
		abovepos2 = pos2.copy()
		abovepos1[2] = abovepos1[2]+self.zrise
		abovepos2[2] = abovepos2[2]+self.zrise

		#go to abovepos1
		self.goto(abovepos1)
		#open gripper
		self.open_gripper()
		#go to pos1
		self.goto(pos1)
		#close gripper
		self.close_gripper()
		#go to abovepos1
		self.goto(abovepos1)
		#go to abovepos2
		self.goto(abovepos2)
		#go to pos2
		self.goto(pos2)
		#open gripper
		self.open_gripper()
		#go to abovepos2
		self.goto(abovepos2)

		rospy.loginfo("Moved item at "+str(pos1)+" to "+str(pos2))
	
	def open_gripper(self)
		#publish the command to open gripper to trajectory planner
		
		#wait for confirmation that gripper is open
		#rospy.wait_for_message(topic, messagetype)
		print (rospy.wait_for_message("/finished_trajectory", Bool))

	def close_gripper(self)
		#publish the command to close gripper to trajectory planner
		
		#wait for confirmation that gripper is closed
		print (rospy.wait_for_message("/finished_trajectory", Bool))

	def goto(self, position)
		#publish the command to go to the position and rotate gripper
		
		#wait for confirmation that we have reach the commanded position
		print (rospy.wait_for_message("/finished_trajectory", Bool))

if __name__ == "__main__":
	rospy.init_node('task_planner', anonymous=True) # Initialize the node
	#rospy.spin() # Keeps python from exiting until the ROS node is stopped

