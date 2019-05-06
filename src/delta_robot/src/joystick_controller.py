#!/usr/bin/env python

import rospy
import sensor_msgs.msg as smsg
import math
import numpy as np
from IK_solver import IKSolver
from workspace_checker import WorkspaceChecker
from std_msgs.msg import Float32MultiArray


class JoystickController:
	def __init__(self):
		self.rate = 100 #[Hz]

		self.sub_joy_right = rospy.Subscriber('joy_right', smsg.Joy, self.cb_joy_right)
		self.sub_joy_left = rospy.Subscriber('joy_left', smsg.Joy, self.cb_joy_left)
		self.joint_commands_pub = rospy.Publisher("/joint_commands", Float32MultiArray, queue_size=10)

		self.xyz = [0,0,-714.66] # Calibrated position
		self.xyz_vel = [0.0,0.0,0.0]

		self.iksolver = IKSolver()
		self.wschecker = WorkspaceChecker()

		rospy.Timer(rospy.Duration(1./self.rate), self.update)

	def cb_joy_right(self, joy):
		self.xyz_vel[0] = 25.*joy.axes[1]
		self.xyz_vel[1] = 25.*joy.axes[0]

	def cb_joy_left(self, joy):
		self.xyz_vel[2] = 25.*joy.axes[1]

	def update(self,event):
		print "Pos: " + str(self.xyz)
		print "Vel: " + str(self.xyz_vel)
		x = self.xyz[0] + self.xyz_vel[0]/self.rate
		y = self.xyz[1] + self.xyz_vel[1]/self.rate
		z = self.xyz[2] + self.xyz_vel[2]/self.rate

		if self.wschecker.check([x,y,z]):
			self.xyz = [x,y,z]
			theta = self.iksolver.solve(self.xyz)
			joint_commands_msg = Float32MultiArray()
			joint_commands_msg.data = theta
			self.joint_commands_pub.publish(joint_commands_msg)

if __name__ == "__main__":
    rospy.init_node('joystick_controller', anonymous=True)
    jc = JoystickController()

    rospy.spin()
