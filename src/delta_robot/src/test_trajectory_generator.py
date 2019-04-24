#!/usr/bin/python

import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from delta_robot.msg import JointPosition
from delta_robot.msg import GripperPosition
from delta_robot.msg import DeltaTrajectory
import time

class TestTrajectoryGenerator:
    def __init__(self):

        self.rate = 100 #[Hz]

        rospy.Subscriber("/finished_trajectory", Bool, self.finished_trajectory_cb)

        self.trajectory_pub = rospy.Publisher("/trajectory", DeltaTrajectory, queue_size=10)

        time.sleep(1)

        self.sine_wave(0.5, 0.5, 5, True)


    def finished_trajectory_cb(self,msg):
        print "Trajectory finished: " + str(msg.data)

    def sine_wave(self, amplitude = 0.2, frequency = 0.5, cycles = 1, include_gripper = False):
        trajectory_msg = DeltaTrajectory()

        joint_traj = []
        gripper_traj = []

        for t in range(0,int(cycles*self.rate/frequency)):
            s = amplitude*math.sin(2*math.pi*t*frequency/self.rate)
            j = JointPosition()
            j.angles = [s,s,s]
            joint_traj.append(j)
            g = GripperPosition()
            g.angle = s
            g.open = True if (t*frequency*1.0/self.rate)%1 > 0.5 else False
            gripper_traj.append(g)

        trajectory_msg.joint_trajectory = joint_traj

        trajectory_msg.gripper_trajectory = gripper_traj

        self.trajectory_pub.publish(trajectory_msg)

if __name__ == "__main__":
    rospy.init_node('test_trajectory_generator', anonymous=True)
    ttg = TestTrajectoryGenerator()

    rospy.spin()

