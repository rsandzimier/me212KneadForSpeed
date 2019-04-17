#!/usr/bin/python

import rospy
import math
import motor
from std_msgs.msg import Float32MultiArray
import odrive


class Odrive:
    def __init__(self):

        self.rate = 100 #[Hz]

        rospy.Subscriber("/joint_commands", Float32MultiArray, self.angle_cb)
        rospy.Subscriber("/calibration_offsets", Float32MultiArray, self.calibration_cb)

        self.joint_angles_pub = rospy.Publisher("/joint_angles", Float32MultiArray, queue_size=10)
        self.joint_currents_pub = rospy.Publisher("/joint_currents", Float32MultiArray, queue_size=10)

        print "Initializing motors"
        odrv0 = odrive.find_any(serial_number = "205337853548")
        odrv1 = odrive.find_any(serial_number = "208737993548")
        self.motor0 = motor.Motor(odrv0,0)
        self.motor1 = motor.Motor(odrv1,0)
        self.motor2 = motor.Motor(odrv1,1)
        print "Finished Initializing motors"

        rospy.Timer(rospy.Duration(1./self.rate), self.publish)

    def angle_cb(self,msg):
        self.motor0.set_angle(msg.data[0])
        self.motor1.set_angle(msg.data[1])
        self.motor2.set_angle(msg.data[2])

    def calibration_cb(self,msg):
        self.motor0.set_calibration(msg.data[0])
        self.motor1.set_calibration(msg.data[1])
        self.motor2.set_calibration(msg.data[2])

    def publish(self,event):
        joint_angles_msg = Float32MultiArray()
        joint_currents_msg = Float32MultiArray()

        joint_angles_msg.data = [self.motor0.get_angle(), self.motor1.get_angle(), self.motor2.get_angle()]
        joint_currents_msg.data = [self.motor0.get_current(), self.motor1.get_current(), self.motor2.get_current()]

        self.joint_angles_pub.publish(joint_angles_msg)
        self.joint_currents_pub.publish(joint_currents_msg)


if __name__ == "__main__":
    rospy.init_node('odrive', anonymous=True)
    od = Odrive()

    rospy.spin()

