#!/usr/bin/python

import rospy
import math
from std_msgs.msg import Float32MultiArray
import time

class Calibration:
    JOINT_SPEED = -0.1 # Radians per second
    CURRENT_THRESHOLD = 4 # Amps
    TARGET_JOINT_POSITION = 0 # Position to go to after calibration is over
    TARGET_POSITION_TOLERANCE = 0.01 # Radians
    MAX_DISTANCE = 2*math.pi # Radians

    HARD_STOP_POSITION = -70*math.pi/180 # Radians, -70 degrees

    def __init__(self):
        self.rate = 100 #[Hz]

        rospy.Subscriber("/joint_angles", Float32MultiArray, self.angle_cb)
        rospy.Subscriber("/joint_currents", Float32MultiArray, self.current_cb)

        self.joint_commands_pub = rospy.Publisher("/calib_joint_commands", Float32MultiArray, queue_size=10)
        self.calibration_offsets_pub = rospy.Publisher("/calibration_offsets", Float32MultiArray, queue_size=10)

        self.calibration_offsets = [0,0,0]
        self.calibration_complete = [False, False, False]

        self.joint_angles = []
        self.joint_currents = []

        self.joint_commands = []

        self.initial_joint_angles = []

        rospy.Timer(rospy.Duration(1./self.rate), self.update)

    def angle_cb(self,msg):
        self.joint_angles = list(msg.data)
        if len(self.joint_commands) == 0:
            self.joint_commands = list(self.joint_angles)
        if len(self.initial_joint_angles) == 0:
            self.initial_joint_angles = list(self.joint_angles)

    def current_cb(self,msg):
        self.joint_currents = list(msg.data)
        for i in range(0,3):
            if self.joint_currents[i] >= self.CURRENT_THRESHOLD and not self.calibration_complete[i]:
                self.calibration_offsets[i] = self.joint_angles[i] - self.HARD_STOP_POSITION
                self.calibration_complete[i] = True
                print "Motor " + str(i) + " Calibrated"

    def update(self,event):
        if len(self.joint_angles) == 0 or len(self.joint_currents) == 0: return

        if all(self.calibration_complete) and \
            abs(self.joint_angles[0] - self.TARGET_JOINT_POSITION - self.calibration_offsets[0]) < self.TARGET_POSITION_TOLERANCE and \
            abs(self.joint_angles[1] - self.TARGET_JOINT_POSITION - self.calibration_offsets[1]) < self.TARGET_POSITION_TOLERANCE and \
            abs(self.joint_angles[2] - self.TARGET_JOINT_POSITION - self.calibration_offsets[2]) < self.TARGET_POSITION_TOLERANCE:
            calibration_offsets_msg = Float32MultiArray()
            calibration_offsets_msg.data = self.calibration_offsets
            self.calibration_offsets_pub.publish(calibration_offsets_msg)
            print "Calibration Complete"
            rospy.signal_shutdown("Calibration Complete")

        joint_commands_msg = Float32MultiArray()

        for i in range(0,3):
            if self.calibration_complete[i]:
                self.joint_commands[i] = self.joint_commands[i] - self.JOINT_SPEED/self.rate
                if self.joint_commands[i] > self.TARGET_JOINT_POSITION + self.calibration_offsets[i]:
                    self.joint_commands[i] = self.TARGET_JOINT_POSITION + self.calibration_offsets[i]
            else:
                self.joint_commands[i] = self.joint_commands[i] + self.JOINT_SPEED/self.rate
        joint_commands_msg.data = self.joint_commands

        self.joint_commands_pub.publish(joint_commands_msg)

        if abs(self.joint_commands[0] - self.initial_joint_angles[0]) > self.MAX_DISTANCE or \
            abs(self.joint_commands[1] - self.initial_joint_angles[1]) > self.MAX_DISTANCE or \
            abs(self.joint_commands[2] - self.initial_joint_angles[2]) > self.MAX_DISTANCE:
            rospy.signal_shutdown("Calibration Failed")

if __name__ == "__main__":
    rospy.init_node('calibration', anonymous=True)
    c = Calibration()

    rospy.spin()

