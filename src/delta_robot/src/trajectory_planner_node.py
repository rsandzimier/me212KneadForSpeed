#!/usr/bin/python

import rospy
import math
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Bool 
from delta_robot.msg import KFSPoseArray
from delta_robot.msg import KFSPose
from delta_robot.msg import DeltaTrajectory
from delta_robot.msg import GripperPosition
from delta_robot.msg import JointPosition
from IK_solver import IKSolver
from workspace_checker import WorkspaceChecker

import numpy as np

class TrajectoryPlanner: 
	XYZ_VEL = 25.0
	XYZ_ACCEL = 9800.0

	ZOFFSET = 50.0

	def __init__(self): 
		self.rate = 100.0 #[Hz]

        self.iksolver = IKSolver()
        self.wschecker = WorkspaceChecker()
        self.xyz_pos_init = [0.0,0.0,-714.66]
		self.xyz_pos = [0.0,0.0,-714.66] # Initial position right after calibration (maybe make this more robust)

		self.running_task = False

		rospy.Subscriber("/place_topping", KFSPoseArray, self.place_topping_cb)
		rospy.Subscriber("/shake_salt", KFSPoseArray, self.shake_salt_cb)
		rospy.Subscriber("/press_dough", KFSPoseArray, self.press_dough_cb)
		rospy.Subscriber("/push_pizza", KFSPose, self.push_pizza_cb)
		rospy.Subscriber("/move_to", KFSPose, self.move_to_cb)
		rospy.Subscriber("/finished_trajectory",Bool, self.finished_move)
		#rospy.Subscriber("/move_to", KFSPose, self.move_to_cb)

		self.finished_task_pub = rospy.Publisher("/finished_task", Bool, queue_size=10)
		self.trajectory_pub = rospy.Publisher("/trajectory", DeltaTrajectory, self.trajectory_cb)

		rospy.Timer(rospy.Duration(1./self.rate), self.function_to_loop)

	def finished_move(self, msg):
		self.running_task = False

	def place_topping_cb(self,msg):
		print("Moving topping")
		topping_xyz=[0,0,0]
		topping_orientation=0
		topping_xyz[0]=msg.poses[0].position.x
		topping_xyz[1]=msg.poses[0].position.y
		topping_xyz[2]=msg.poses[0].position.z
		topping_orientation=msg.poses[0].orientation

		above_topping_xyz=[0,0,0]
		above_topping_orientation=0
		above_topping_xyz[0]=msg.poses[0].position.x
		above_topping_xyz[1]=msg.poses[0].position.y
		above_topping_xyz[2]=msg.poses[0].position.z+self.ZOFFSET
		above_topping_orientation=msg.poses[0].orientation

		dest_xyz=[0,0,0]
		dest_orientation=0
		dest_xyz[0]=msg.poses[1].position.x
		dest_xyz[1]=msg.poses[1].position.y
		dest_xyz[2]=msg.poses[1].position.z
		dest_orientation=msg.poses[1].orientation

		above_dest_xyz=[0,0,0]
		above_dest_orientation=0
		above_dest_xyz[0]=msg.poses[1].position.x
		above_dest_xyz[1]=msg.poses[1].position.y
		above_dest_xyz[2]=msg.poses[1].position.z+self.ZOFFSET
		above_dest_orientation=msg.poses[1].orientation

		generateMoveTo(above_topping_xyz,above_topping_orientation,True)
		generateMoveTo(topping_xyz,topping_orientation,True)
		#directly publish to close gripper
		generateMoveTo(above_topping_xyz,above_topping_orientation,False)
		generateMoveTo(above_dest_xyz, above_dest_orientation, False)
		generateMoveTo(dest_xyz, dest_orientation, False)
		#directly publush to open
		generateMoveTo(self.xyz_pos_init, 0, True)
		print("Moved topping!")

	def shake_salt_cb(self,msg): 
		pass
	def press_dough_cb(self,msg): 
		pass
	def push_pizza_cb(self,msg):
		XOFFSET = 200

		dest_xyz=[0,0,0]
		dest_orientation=0
		dest_xyz[0]=msg.position.x+XOFFSET
		dest_xyz[1]=msg.position.y
		dest_xyz[2]=msg.position.z
		dest_orientation=msg.orientation
		generateMoveTo(topping_xyz,topping_orientation,False)

		dest_xyz[0] = dest_xyz[0]-3*XOFFSET
		generateMoveTo(topping_xyz,topping_orientation,False)

	def move_to_cb(self,msg): 
		dest_xyz=[0,0,0]
		dest_orientation=0
		dest_xyz[0]=msg.position.x
		dest_xyz[1]=msg.position.y
		dest_xyz[2]=msg.position.z
		dest_orientation=msg.orientation
		dest_open = msg.open

		generateMoveTo(topping_xyz,topping_orientation,dest_open)

	def generateMoveTo(self, xyz, orientation, gripper_open)
		if (self.running_task == True):
			print("waiting for previous task to finish")
			rospy.wait_for_message("/finished_trajectory", Bool)
			self.running_task = False

		self.running_task = True
		desired_xyz = np.array(xyz)
		desired_rot = orientation
		desired_open = gripper_open

		xyz = np.array(self.xyz_pos)
		diff = desired_xyz - xyz
		d = np.linalg.norm(diff)

		tr = XYZ_VEL/XYZ_ACCEL
		tm = d/XYZ_VEL - tr
		if tm < 0:
			tr = math.sqrt(2*d/XYZ_ACCEL)
			tm = 0

		t_tot = tm + 2*tr

		t_ = [i*1./self.rate for i in (0,int(t_tot*self.rate))]

		trajectory_msg = DeltaTrajectory()
        joint_traj = []
        gripper_traj = []

        for t in t_:
			j = JointPosition()
			s = 0
			if t < tr:
				s = 0.5*XYZ_ACCEL*t**2
			elif t < tr+tm:
				s = 0.5*XYZ_ACCEL*tr**2 + (t - tr)*XYZ_VEL
			else:
				s = 0.5*XYZ_ACCEL*tr**2 + tm*XYZ_VEL + 0.5*XYZ_ACCEL*(tr**2 - (t - tm - 2*tr)**2)
			#Interpolates directly from A to B, no hopping motion
			xyz_i = xyz + diff*s/d
			if self.wschecker.check(xyz_i):
				# DO IK
				j.data=self.iksolver.solve(xyz_i)
				#set j.data =
			else: 
				# print error 
				print("error not in workspace")
				return
			g = GripperPosition()
			g.open = gripper_open
			g.angle = orientation
			
			joint_traj.append(j)
			gripper_traj.append(g)

		trajectory_msg.joint_trajectory = joint_traj
		trajectory_msg.gripper_trajectory = gripper_traj

		self.trajectory_pub.publish(trajectory_msg)





if __name__ == "__main__":
	rospy.init_node('trajectory_planner', anonymous=True) # Initialize the node
	tp = TrajectoryPlanner() # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 

	rospy.spin() # Keeps python from exiting until the ROS node is stopped

