#!/usr/bin/python

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from delta_robot.msg import GripperPosition
from delta_robot.msg import DeltaTrajectory

class TrajectoryFollower:
    TARGET_POSITION_TOLERANCE = 0.01 # Radians

    def __init__(self):

        self.rate = 100 #[Hz]

        rospy.Subscriber("/joint_angles", Float32MultiArray, self.joint_angle_cb)
        rospy.Subscriber("/gripper_state", GripperPosition, self.gripper_state_cb)
        rospy.Subscriber("/trajectory", DeltaTrajectory, self.trajectory_cb)

        self.joint_commands_pub = rospy.Publisher("/joint_commands", Float32MultiArray, queue_size=10)
        self.gripper_commands_pub = rospy.Publisher("/gripper_commands", GripperPosition, queue_size=10)
        self.finished_trajectory_pub = rospy.Publisher("/finished_trajectory", Bool, queue_size=10)

        self.joint_angles = [0.0,0.0,0.0]
        self.gripper_angle = 0.0
        self.gripper_open = False

        self.reset_traj()

        rospy.Timer(rospy.Duration(1./self.rate), self.update)

    def joint_angle_cb(self,msg):
        self.joint_angles = msg.data

    def gripper_state_cb(self,msg):
        self.gripper_angle = msg.angle
        self.gripper_open = msg.open

    def trajectory_cb(self,msg):
        self.reset_traj()
        self.joint_trajectory = msg.joint_trajectory
        self.gripper_trajectory = msg.gripper_trajectory
        if len(self.joint_trajectory) != len(self.gripper_trajectory):
            self.reset_traj()
            print "Error: Joint and gripper trajectories must be same length"
            return

    def update(self,event):
        if self.current_step >= len(self.joint_trajectory):
            if len(self.joint_trajectory) > 0 and self.reached_last_frame():
                finished_trajectory_msg = Bool()
                finished_trajectory_msg.data = True
                self.finished_trajectory_pub.publish(finished_trajectory_msg)
                self.reset_traj()
            return

        joint_commands_msg = Float32MultiArray()
        gripper_commands_msg = GripperPosition()

        joint_commands_msg.data = self.joint_trajectory[self.current_step].angles
        gripper_pos = self.gripper_trajectory[self.current_step]
        gripper_commands_msg.angle = gripper_pos.angle
        gripper_commands_msg.open = gripper_pos.open

        self.joint_commands_pub.publish(joint_commands_msg)
        self.gripper_commands_pub.publish(gripper_commands_msg)
        self.current_step += 1

    def reset_traj(self):
        self.joint_trajectory = []
        self.gripper_trajectory = []
        self.current_step = 0

    def reached_last_frame(self):
        if abs(self.joint_angles[0] - self.joint_trajectory[-1].angles[0]) < self.TARGET_POSITION_TOLERANCE and \
            abs(self.joint_angles[1] - self.joint_trajectory[-1].angles[1]) < self.TARGET_POSITION_TOLERANCE and \
            abs(self.joint_angles[2] - self.joint_trajectory[-1].angles[2]) < self.TARGET_POSITION_TOLERANCE and \
            abs(self.gripper_angle - self.gripper_trajectory[-1].angle)*0.1 < self.TARGET_POSITION_TOLERANCE and \
            self.gripper_open == self.gripper_trajectory[-1].open:
            return True
        return False

if __name__ == "__main__":
    rospy.init_node('trajectory_follower', anonymous=True)
    tf = TrajectoryFollower()

    rospy.spin()

