#!/usr/bin/env python

import rospy
import numpy as np
import tf
import time
import utils

from geometry_msgs.msg import PolygonStamped, Pose
from visualization_msgs.msg import Marker
from mobile_robot.msg import WheelCmdVel
from nav_msgs.msg import Odometry

class PurePursuit(object):
	""" Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
	"""
	def __init__(self):
		self.trajectory_topic = rospy.get_param("~trajectory_topic")
		self.base_frame       = "base_link"
		self.lookahead        = .15
		self.speed            = .3
		self.wrap             = bool(rospy.get_param("~wrap"))
		self.wheelbase_length = 0.44
		self.drive_topic      = "/cmdvel"
		self.trajectory_set   = False
		self.wheel_radius = 0.037

		self.trajectory  = utils.LineTrajectory("/followed_trajectory")
		self.traj_sub = rospy.Subscriber(self.trajectory_topic, PolygonStamped, self.trajectory_callback, queue_size=1)
		self.listener = tf.TransformListener()
		self.broadcaster = tf.TransformBroadcaster()

		self.pt_pub = rospy.Publisher("robot_point", Marker, queue_size=10)
		self.circle_pub = rospy.Publisher("lookahead_circle", Marker, queue_size=10)
		self.drive_pub = rospy.Publisher(self.drive_topic, WheelCmdVel, queue_size=10)

		rospy.Timer(rospy.Duration(1./30), self.pose_callback)

	def pose_callback(self, msg):
		try:
			trans, rot = self.listener.lookupTransform('/map', self.base_frame, rospy.Time(0))
			x = trans[0]
			y = trans[1]
			th = tf.transformations.euler_from_quaternion(rot)
			self.pursue([x, y, th])
		except tf.LookupException:
			print("lookup")

	def trajectory_callback(self, msg):
		''' Clears the currently followed trajectory, and loads the new one from the message
		'''
		print "Receiving new trajectory:", len(msg.polygon.points), "points" 
		self.trajectory.clear()
		self.trajectory.fromPolygon(msg.polygon)
		self.trajectory.publish_viz(duration=0.0)
		self.trajectory_set = True

	def pursue(self, car_pos):
		if not self.trajectory_set:
			return
		print("pursue")
		x, y, th = car_pos
		point_found, carrot_pos = self.trajectory.find_drive_point(np.array([x, y]), self.lookahead)
		pose = Pose()
		pose.position.x = carrot_pos[0]
		pose.position.y = carrot_pos[1]
		quaternions = (0.0, 0.0, 0.0, 1.0)
		pose.orientation.x = quaternions[0]
		pose.orientation.y = quaternions[1]
		pose.orientation.z = quaternions[2]
		pose.orientation.w = quaternions[3]
		robotMarker = Marker()
		robotMarker.type = Marker.SPHERE
		robotMarker.header.frame_id = "map"
		robotMarker.scale.x = 0.1
		robotMarker.scale.y = 0.1
		robotMarker.scale.z = 0.1
		robotMarker.color.a = 1.0
		robotMarker.color.r = 1.0
		robotMarker.color.g = 0.5
		robotMarker.pose = pose
		self.pt_pub.publish(robotMarker)

		cpose = Pose()
		cpose.position.x = x
		cpose.position.y = y
		quaternions = (0.0, 0.0, 0.0, 1.0)
		cpose.orientation.x = quaternions[0]
		cpose.orientation.y = quaternions[1]
		cpose.orientation.z = quaternions[2]
		cpose.orientation.w = quaternions[3]
		cylMarker = Marker()
		cylMarker.type = Marker.CYLINDER
		cylMarker.header.frame_id = "map"
		cylMarker.scale.x = self.lookahead*2
		cylMarker.scale.y = self.lookahead*2
		cylMarker.scale.z = 0.0
		cylMarker.color.a = 0.5
		cylMarker.color.g = 1.0
		cylMarker.pose = cpose
		self.circle_pub.publish(cylMarker)

		self.broadcaster.sendTransform((carrot_pos[0], carrot_pos[1], 0.0),
 									   (0.0, 0.0, 0.0, 1.0),
									   rospy.Time.now(),
									   "carrot",
									   "map")
		try:
			trans, rot = self.listener.lookupTransform(self.base_frame, "/carrot", rospy.Time(0))
			carrot_y = trans[1]
			curvature = 2 * carrot_y / self.lookahead**2
			if point_found:
				self.send_commands(curvature, self.speed)
			else:
				self.send_commands(curvature, 0.0)
		except:
			pass

	def send_commands(self, curvature, speed):
		drive_msg = WheelCmdVel()
		drive_msg.desiredWV_R = speed * (1+curvature*self.wheelbase_length/2)
		drive_msg.desiredWV_L = speed * (1-curvature*self.wheelbase_length/2)
		self.drive_pub.publish(drive_msg)




if __name__=="__main__":
	rospy.init_node("pure_pursuit")
	pf = PurePursuit()
	rospy.spin()
