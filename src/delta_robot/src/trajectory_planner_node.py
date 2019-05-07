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
	XYZ_VEL = 100.0
    XYZ_ACCEL = 4900.0
    XYZ_VEL_FAST = 100.0
    XYZ_ACCEL_FAST = 4900.0
    pizzaradius=115.0
    shake_distance=10.0
	ZOFFSET = 50.0

	def __init__(self): 
		self.rate = 100.0 #[Hz]

		self.iksolver = IKSolver()
		self.wschecker = WorkspaceChecker()
		self.xyz_pos_init = [0.0,0.0,-714.66]
		self.xyz_pos = [0.0,0.0,-714.66] # Initial position right after calibration (maybe make this more robust)
		self.xyz_goal_pos = [0.0,0.0,-714.66]

		self.running_task = False
		self.running_trajectory = False

		rospy.Subscriber("/place_topping", KFSPoseArray, self.place_topping_cb)
		rospy.Subscriber("/shake_salt", KFSPoseArray, self.shake_salt_cb)
		rospy.Subscriber("/press_dough", KFSPoseArray, self.press_dough_cb)
		rospy.Subscriber("/push_pizza", KFSPose, self.push_pizza_cb)
		rospy.Subscriber("/move_to", KFSPose, self.move_to_cb)
		rospy.Subscriber("/finished_trajectory", Bool, self.finished_trajectory_cb)

		self.finished_task_pub = rospy.Publisher("/finished_task", Bool, queue_size=10)
		self.trajectory_pub = rospy.Publisher("/trajectory", DeltaTrajectory, queue_size=10)

	def finished_trajectory_cb(self, msg):
		self.running_trajectory = not msg.data
		if not self.running_trajectory:
			self.xyz_pos = self.xyz_goal_pos
		else:
			print "Trajectory Follower published False for /finished_trajectory topic."

	def place_topping_cb(self,msg):
		if self.running_task:
			print "Already working on a task"
			return
		self.running_task = True
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

		self.generateMoveTo(above_topping_xyz,above_topping_orientation,True)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(topping_xyz,topping_orientation,True)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(topping_xyz,topping_orientation,False)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(above_topping_xyz,above_topping_orientation,False)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(above_dest_xyz, above_dest_orientation, False)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(dest_xyz, dest_orientation, False)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(dest_xyz,dest_orientation,True)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(self.xyz_pos_init, 0, True)
		self.waitForTrajectoryToFinish()
		print("Moved topping!")
		self.running_task = False
		finished_task_msg = Bool()
		finished_task_msg.data = True
		self.finished_task_pub.publish(finished_task_msg)

	def shake_salt_cb(self,msg): 
		if self.running_task:
			print "Already working on a task"
			return
        self.running_task = True
        shaker_xyz=[0,0,0]
		shaker_orientation=0
		shaker_xyz[0]=msg.poses[0].position.x
		shaker_xyz[1]=msg.poses[0].position.y
		shaker_xyz[2]=msg.poses[0].position.z
        shaker_orientation=msg.poses[0].orientation
        
        above_shaker_xyz=[0,0,0]
		above_shaker_orientation=0
		above_shaker_xyz[0]=msg.poses[0].position.x
		above_shaker_xyz[1]=msg.poses[0].position.y
		above_shaker_xyz[2]=msg.poses[0].position.z+self.ZOFFSET
        above_shaker_orientation=msg.poses[0].orientation
        
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
        
        

        above_dest_left_xyz=[0,0,0]
		above_dest_orientation=0
		above_dest_left_xyz[0]=msg.poses[1].position.x-self.shake_distance
		above_dest_left_xyz[1]=msg.poses[1].position.y
		above_dest_left_xyz[2]=msg.poses[1].position.z+self.ZOFFSET
		above_dest_left_orientation=msg.poses[1].orientation

		above_dest_right_xyz=[0,0,0]
		above_dest_right_orientation=0
		above_dest_right_xyz[0]=msg.poses[1].position.x+self.shake_distance
		above_dest_right_xyz[1]=msg.poses[1].position.y
		above_dest_right_xyz[2]=msg.poses[1].position.z+self.ZOFFSET
		above_dest_right_orientation=msg.poses[1].orientation
        
        self.generateMoveTo(above_shaker_xyz,above_shaker_orientation,True)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(shaker_xyz,shaker_orientation,True)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(shaker_xyz,shaker_orientation,False)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(above_shaker_xyz,above_shaker_orientation,False)
        self.waitForTrajectoryToFinish()
        print('grabbed shaker')
        
		self.generateMoveTo(above_dest_xyz, above_dest_orientation, False)
        self.waitForTrajectoryToFinish()
        
		self.generateMoveToFAST(above_dest_left_xyz, dest_orientation, False)
        self.waitForTrajectoryToFinish()
        self.generateMoveToFAST(above_dest_right_xyz, dest_orientation, False)
        self.waitForTrajectoryToFinish()
        print('im shook for the first time')
        self.generateMoveToFAST(above_dest_left_xyz, dest_orientation, False)
        self.waitForTrajectoryToFinish()
        self.generateMoveToFAST(above_dest_right_xyz, dest_orientation, False)
        self.waitForTrajectoryToFinish()
        print('im shook for the second time')

        self.generateMoveTo(above_dest_xyz, above_dest_orientation, False)
        self.waitForTrajectoryToFinish()
        
        self.generateMoveTo(above_shaker_xyz,above_shaker_orientation,False)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(shaker_xyz,shaker_orientation,False)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(shaker_xyz,shaker_orientation,True)
		self.waitForTrajectoryToFinish()
		self.generateMoveTo(above_shaker_xyz,above_shaker_orientation,True)
        self.waitForTrajectoryToFinish()
        
        print("Shook salt!")
		self.running_task = False
		finished_task_msg = Bool()
		finished_task_msg.data = True
		self.finished_task_pub.publish(finished_task_msg)
        

	def press_dough_cb(self,msg): 
        if self.running_task:
            print "Already working on a task"
            return
        self.running_task = True
        presser_xyz=[0,0,0]
        presser_orientation=0
        presser_xyz[0]=msg.poses[0].position.x
        presser_xyz[1]=msg.poses[0].position.y
        presser_xyz[2]=msg.poses[0].position.z
        presser_orientation=msg.poses[0].orientation

        above_presser_xyz=[0,0,0]
        above_presser_orientation=0
        above_presser_xyz[0]=msg.poses[0].position.x
        above_presser_xyz[1]=msg.poses[0].position.y
        above_presser_xyz[2]=msg.poses[0].position.z+self.ZOFFSET
        above_presser_orientation=msg.poses[0].orientation

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


        self.generateMoveTo(above_presser_xyz,above_presser_orientation,True)
        self.waitForTrajectoryToFinish()
        self.generateMoveTo(presser_xyz,presser_orientation,True)
        self.waitForTrajectoryToFinish()
        self.generateMoveTo(presser_xyz,presser_orientation,False)
        self.waitForTrajectoryToFinish()
        print('grabbed presser')
        self.generateMoveTo(above_presser_xyz,above_presser_orientation,False)
        self.waitForTrajectoryToFinish()
        theta=0.0
        while theta<360.0:
            r=self.pizzaradius*theta/360.0
            x_off=r*math.cos(math.radians(theta))
            y_off=r*math.sin(math.radians(theta))
            print([x_off,y_off,theta])
            dest_xyz[0]=msg.poses[1].position.x+x_off
            dest_xyz[1]=msg.poses[1].position.y+y_off
            dest_xyz[2]=msg.poses[1].position.z
            above_dest_xyz[0]=msg.poses[1].position.x+x_off
            above_dest_xyz[1]=msg.poses[1].position.y+y_off
            above_dest_xyz[2]=msg.poses[1].position.z+self.ZOFFSET
            self.generateMoveTo(above_dest_xyz, above_dest_orientation, False)
            self.waitForTrajectoryToFinish()
            self.generateMoveTo(dest_xyz, dest_orientation, False)
            self.waitForTrajectoryToFinish()
            self.generateMoveTo(above_dest_xyz, above_dest_orientation, False)
            self.waitForTrajectoryToFinish()
            theta+=60.0

        self.generateMoveTo(above_presser_xyz, above_presser_orientation, False)
        self.waitForTrajectoryToFinish()
        self.generateMoveTo(presser_xyz,presser_orientation,False)
        self.waitForTrajectoryToFinish()
        self.generateMoveTo(presser_xyz,presser_orientation,True)
        self.waitForTrajectoryToFinish()
        self.generateMoveTo(above_presser_xyz,above_presser_orientation,True)
        self.waitForTrajectoryToFinish()

        print("Pressed Dough!")
        self.running_task = False
        finished_task_msg = Bool()
        finished_task_msg.data = True
        self.finished_task_pub.publish(finished_task_msg)
        

	def push_pizza_cb(self,msg):
		if self.running_task:
			print("Already working on a task")
			return
		self.running_task = True
        
		dest_xyz=[0,0,0]
		dest_orientation=0
		dest_xyz[0]=msg.position.x-self.pizzaradius
		dest_xyz[1]=msg.position.y
		dest_xyz[2]=msg.position.z
		dest_orientation=msg.orientation
		self.generateMoveTo(dest_xyz,dest_orientation,False)
		self.waitForTrajectoryToFinish()
		dest_xyz[0] = dest_xyz[0]+3*self.pizzaradius
		self.generateMoveTo(dest_xyz,dest_orientation,False)
        self.waitForTrajectoryToFinish()
        print("Pushed Pizza!")
		self.running_task = False
		finished_task_msg = Bool()
		finished_task_msg.data = True
		self.finished_task_pub.publish(finished_task_msg)

	def move_to_cb(self,msg): 
		if self.running_task:
			print "Already working on a task"
			return
		self.running_task = True
		print("Running move_to")
		dest_xyz=[0,0,0]
		dest_orientation=0
		dest_xyz[0]=msg.position.x
		dest_xyz[1]=msg.position.y
		dest_xyz[2]=msg.position.z
		dest_orientation=msg.orientation
		dest_open = msg.open
		self.generateMoveTo(dest_xyz,dest_orientation,dest_open)
		self.waitForTrajectoryToFinish()
		self.running_task = False
		finished_task_msg = Bool()
		finished_task_msg.data = True
        self.finished_task_pub.publish(finished_task_msg)
        
    def generateMoveToFAST(self,xyz,orientation,gripper_open):
        if self.running_trajectory:
        print("Currently running a trajectory. Cannot call this function until goal position reached")
        return

        self.running_task = True
        desired_xyz = np.array(xyz)
        desired_rot = orientation
        desired_open = gripper_open

        xyz_initial = np.array(self.xyz_pos)
        diff = desired_xyz - xyz_initial
        d = np.linalg.norm(diff)

        tr = self.XYZ_VEL_FAST/self.XYZ_ACCEL_FAST
        tm = d/self.XYZ_VEL_FAST - tr
        if tm < 0:
            tr = math.sqrt(2*d/self.XYZ_ACCEL_FAST)
            tm = 0

        t_tot = tm + 2*tr

        t_ = [i*1./self.rate for i in range(0,int(t_tot*self.rate))]

        trajectory_msg = DeltaTrajectory()
        joint_traj = []
        gripper_traj = []

        for t in t_:
            j = JointPosition()
            s = 0
            if t < tr:
                s = 0.5*self.XYZ_ACCEL_FAST*t**2
            elif t < tr+tm:
                s = 0.5*self.XYZ_ACCEL_FAST*tr**2 + (t - tr)*self.XYZ_VEL_FAST
            else:
                s = 0.5*self.XYZ_ACCEL_FAST*tr**2 + tm*self.XYZ_VEL_FAST + 0.5*self.XYZ_ACCEL_FAST*(tr**2 - (t - tm - 2*tr)**2)
            #Interpolates directly from A to B, no hopping motion
            xyz_i = xyz_initial + diff*s/d if d != 0 else xyz_initial
            if self.wschecker.check(xyz_i):
                # DO IK
                j.angles=self.iksolver.solve(xyz_i)
            else: 
                # print error 
                print("error not in workspace")
                return
            g = GripperPosition()
            g.open = gripper_open
            g.angle = orientation
            
            joint_traj.append(j)
            gripper_traj.append(g)

        if len(joint_traj) == 0 and len(gripper_traj) == 0:
            j = JointPosition()
            if self.wschecker.check(xyz_initial):
                # DO IK
                j.angles=self.iksolver.solve(xyz_initial)
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

        self.running_trajectory = True
        self.xyz_goal_pos = list(desired_xyz)
        self.trajectory_pub.publish(trajectory_msg)

	def generateMoveTo(self, xyz, orientation, gripper_open):
		if self.running_trajectory:
			print("Currently running a trajectory. Cannot call this function until goal position reached")
			return

		self.running_task = True
		desired_xyz = np.array(xyz)
		desired_rot = orientation
		desired_open = gripper_open

		xyz_initial = np.array(self.xyz_pos)
		diff = desired_xyz - xyz_initial
		d = np.linalg.norm(diff)

		tr = self.XYZ_VEL/self.XYZ_ACCEL
		tm = d/self.XYZ_VEL - tr
		if tm < 0:
			tr = math.sqrt(2*d/self.XYZ_ACCEL)
			tm = 0

		t_tot = tm + 2*tr

		t_ = [i*1./self.rate for i in range(0,int(t_tot*self.rate))]

		trajectory_msg = DeltaTrajectory()
		joint_traj = []
		gripper_traj = []

		for t in t_:
			j = JointPosition()
			s = 0
			if t < tr:
				s = 0.5*self.XYZ_ACCEL*t**2
			elif t < tr+tm:
				s = 0.5*self.XYZ_ACCEL*tr**2 + (t - tr)*self.XYZ_VEL
			else:
				s = 0.5*self.XYZ_ACCEL*tr**2 + tm*self.XYZ_VEL + 0.5*self.XYZ_ACCEL*(tr**2 - (t - tm - 2*tr)**2)
			#Interpolates directly from A to B, no hopping motion
			xyz_i = xyz_initial + diff*s/d if d != 0 else xyz_initial
			if self.wschecker.check(xyz_i):
				# DO IK
				j.angles=self.iksolver.solve(xyz_i)
			else: 
				# print error 
				print("error not in workspace")
				return
			g = GripperPosition()
			g.open = gripper_open
			g.angle = orientation
			
			joint_traj.append(j)
			gripper_traj.append(g)

		if len(joint_traj) == 0 and len(gripper_traj) == 0:
			j = JointPosition()
			if self.wschecker.check(xyz_initial):
				# DO IK
				j.angles=self.iksolver.solve(xyz_initial)
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

		self.running_trajectory = True
		self.xyz_goal_pos = list(desired_xyz)
		self.trajectory_pub.publish(trajectory_msg)

	def waitForTrajectoryToFinish(self):
		while self.running_trajectory or self.xyz_pos != self.xyz_goal_pos:
			rospy.sleep(1/self.rate)

if __name__ == "__main__":
	rospy.init_node('trajectory_planner', anonymous=True) # Initialize the node
	tp = TrajectoryPlanner() # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 

	rospy.spin() # Keeps python from exiting until the ROS node is stopped


