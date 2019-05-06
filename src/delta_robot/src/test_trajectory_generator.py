#!/usr/bin/python

import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from delta_robot.msg import JointPosition
from delta_robot.msg import GripperPosition
from delta_robot.msg import DeltaTrajectory
import time
from IK_solver import IKSolver
from workspace_checker import WorkspaceChecker

class TestTrajectoryGenerator:
    def __init__(self):

        self.rate = 100 #[Hz]

        rospy.Subscriber("/finished_trajectory", Bool, self.finished_trajectory_cb)

        self.trajectory_pub = rospy.Publisher("/trajectory", DeltaTrajectory, queue_size=10)

        self.iksolver = IKSolver()
        self.wschecker = WorkspaceChecker()

        time.sleep(1)

        self.helix(150, 100, 200, 0.5)

    def finished_trajectory_cb(self,msg):
        print "Trajectory finished: " + str(msg.data)

    def sine_wave(self, amplitude = 0.2, frequency = 0.2, cycles = 1, include_gripper = False):
        trajectory_msg = DeltaTrajectory()

        joint_traj = []
        gripper_traj = []

        for t in range(0,int(cycles*self.rate/frequency)):
            s = amplitude*math.sin(2*math.pi*t*frequency/self.rate)
            j = JointPosition()
            j.angles = [s,s,s]
            joint_traj.append(j)
            g = GripperPosition()
            g.angle = 0
            g.open = False# if (t*frequency*1.0/self.rate)%1 > 0.5 else False
            gripper_traj.append(g)

        trajectory_msg.joint_trajectory = joint_traj

        trajectory_msg.gripper_trajectory = gripper_traj

        self.trajectory_pub.publish(trajectory_msg)

    def helix(self, radius, pitch, h, freq):
        initial_z = -714.66
        current = [0,0,initial_z]
        initial = [radius,0,initial_z]
        trajectory_xyz = []

        for t in range(0,100):
            xyz = [radius*t*1./100, 0, initial_z]
            trajectory_xyz.append(xyz)


        for t in range(0,int(self.rate*h/pitch/freq)):
            xyz = [radius*math.cos(2*math.pi*freq*t/self.rate),radius*math.sin(2*math.pi*freq*t/self.rate), initial_z + freq*pitch*t/self.rate]
            trajectory_xyz.append(xyz)

        for t in range(0,int(self.rate*h/pitch/freq)):
            xyz = [radius*math.cos(2*math.pi*freq*t/self.rate),radius*math.sin(2*math.pi*freq*t/self.rate), initial_z + h - freq*pitch*t/self.rate]
            trajectory_xyz.append(xyz)

        for t in range(0,100):
            xyz = [radius*(1-t*1./100), 0, initial_z]
            trajectory_xyz.append(xyz)

        joint_traj = []
        gripper_traj = []
        for p in trajectory_xyz:
            if self.wschecker.check(p):
                theta = self.iksolver.solve(p)
                j = JointPosition()
                j.angles = theta
                joint_traj.append(j)
                g = GripperPosition()
                gripper_traj.append(g)
            else: 
                print "Trajectory not in workspace"
                return False
        trajectory_msg = DeltaTrajectory()
        trajectory_msg.joint_trajectory = joint_traj
        trajectory_msg.gripper_trajectory = gripper_traj

        self.trajectory_pub.publish(trajectory_msg)


if __name__ == "__main__":
    rospy.init_node('test_trajectory_generator', anonymous=True)
    ttg = TestTrajectoryGenerator()

    rospy.spin()

