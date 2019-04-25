#!/usr/bin/python

# A template to use for making a ROS node. Shows how to publish/subscribe to topics and how to structure the node as a class
# This is not the only way, but for consistency across our team try not to deviate too far so we can all have an easier time reading each other's code

# Import any necessary libraries
import rospy
import math
import Tkinter as tk
#from std_msgs.msg import Float32MultiArray # A standard message type that allows you to use topics that publish float arrays See http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html
# There are more standard messages (std_msgs.msg) available if needed. You can also create your own custom messages if needed.
# You can also import your own python files you write (can help organize code if it is long and you want to break it up into multiple files)

class InterfaceNode(tk.Frame): 

	def __init__(self,parent): # This function is called one time as soon as an instance of a class is created
		self.rate = 3 #[Hz]

		# Declaring a subscriber. Parameters: topic name (string), topic data type, callback function
		# Every time another node publishes a message to "/arbitrary_subscribed_topic_name", we will call the function called self.arbitrary_topic_name_cb
		# The "/" before the topic name is important
		#rospy.Subscriber("/arbitrary_subscribed_topic_name", Float32MultiArray, self.get_state_cb)
		# Declaring a publisher. Parameters: topic name (string), topic data type, queue_size (use queue_size=10 as default)
		#self.delta_state_pub = rospy.Publisher("/arbitrary_published_topic_name", Float32MultiArray, queue_size=10)

		# Do any other initialization of class
		#self.arbitrary_class_variable_name1 = []
		#self.arbitrary_class_variable_name2 = 0
		self.deltaxyz = [0,0,0]
		self.gripperangle = 0
		self.fingerstate = 0

		self.deltasetxyz = [0,0,0]
		self.setgripangle = 0
		self.setfingerstate = 0

		#tk stuff
		tk.Frame.__init__(self, parent);

		self.prompt = tk.Label(self, text="Enter something:", anchor="w")
		self.entry = tk.Entry(self);
		self.submit = tk.Button(self, text="Submit", command = self.publish_state)
		self.output = tk.Label(self, text="Test text")

		self.deltax = tk.Label(self, text="x=0")
		self.deltay = tk.Label(self, text="y=0")
		self.deltaz = tk.Label(self, text="z=0")
		self.griplabel = tk.Label(self, text="Closed")
		self.gripangle = tk.Label(self, text="Gripper Angle=0")

		self.entryx = tk.Entry(self)
		self.entryy = tk.Entry(self)
		self.entryz = tk.Entry(self)
		self.entrya = tk.Entry(self)
		self.actgripo = tk.Radiobutton(self, text="Open", variable=self.setfingerstate, value=1, indicatoron=0)
		self.actgripc = tk.Radiobutton(self, text="Close", variable=self.setfingerstate, value=0, indicatoron=0)

		self.deltax.grid(row=0, column=0)
		self.deltay.grid(row=0, column=1)
		self.deltaz.grid(row=0, column=2)
		self.entryx.grid(row=1, column=0)
		self.entryy.grid(row=1, column=1)
		self.entryz.grid(row=1, column=2)
		self.griplabel.grid(row=2, column=2)
		self.gripangle.grid(row=2, column=0)
		self.entrya.grid(row=3,column=0)
		self.actgripo.grid(row=3, column=1)
		self.actgripc.grid(row=3, column=2)
		self.submit.grid(row=4,column=1)



		#pack everything
		#self.prompt.grid(side="top", fill="x");
		#self.entry.pack(side="top", fill="x", padx=20)
		#self.output.pack(side="top", fill="x", expand=True)
		#self.submit.pack(side="right")

		self.counter = 0

		# Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
		rospy.Timer(rospy.Duration(1./self.rate), self.update_window)

	def publish_state(self):
		#publishes the user defined coordinates to the delta robot
		print "published state!"
		self.deltasetxyz = [float(self.entryx.get()),float(self.entryy.get()), float(self.entryz.get())]
		print self.deltasetxyz
		self.setgripangle = float(self.entrya.get())
		print self.setgripangle
		print self.setfingerstate

	def get_state_cb(self,msg): # the _cb suffix stands for callback. Use this suffix on your callback functions for clarity
		# This function is called every time another node publishes to the topic called "/arbitrary_subscribed_topic_name"
		# msg is the message that was published to "/arbitrary_subscribed_topic_name"
		# Do whatever needs to happen when we receive this message. 
		# For example, set a class variable to the float array in the message and increment another class variable:
		pass

	def update_window(self,event): # This is the function that we set a timer for above. 
		#Update the values in the window
		self.counter = self.counter + 1
		self.deltaxyz[1] = self.counter
		self.deltaxyz[0] = self.counter*2
		self.deltaxyz[2] = self.counter*3+4
		self.gripperangle = self.counter*7-21
		#self.output.config(text=str(self.counter))

		self.deltax.config(text="x="+str(self.deltaxyz[0]))
		self.deltay.config(text="y="+str(self.deltaxyz[1]))
		self.deltaz.config(text="z="+str(self.deltaxyz[2]))

		self.gripangle.config(text="Gripper Angle="+str(self.gripperangle))
		if (self.fingerstate==1):
			self.griplabel.config(text="Open")
		else:
			self.griplabel.config(text="Closed")


if __name__ == "__main__":
	rospy.init_node('interface_node', anonymous=True) # Initialize the node
	root = tk.Tk()
	interface = InterfaceNode(root) # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 
	interface.pack(fill="both", expand=True)
	root.mainloop()
	#rospy.spin() # Keeps python from exiting until the ROS node is stopped

