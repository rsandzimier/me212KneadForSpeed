#!/usr/bin/python

import rospy
import tf
import numpy as np
import tf.transformations as tfm

from apriltags2_ros.msg import AprilTagDetectionArray
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad
from std_msgs.msg import Float32

class deltaCameraAngle:
    def __init__(self):
        self.done = False
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_cb, queue_size = 1)
        self.angle_pub = rospy.Publisher("camera_angle", Float32, queue_size=10)
        self.avg = 0
        self.count = 0
        rospy.sleep(1)
        
        rospy.spin()

    ## apriltag msg handling function (Need to modify for Task 2)
    def apriltag_cb(self, msg):
        try:
            if self.done: return
        except:
            return
        # use apriltag pose detection to find where is the robot
        for detection in msg.detections:
            if detection.id[0] == 10:   # tag id is the correct one
                poselist_tag_cam = pose2poselist(detection.pose.pose.pose)
                euler = tfm.euler_from_quaternion(poselist_tag_cam[3:7])
                self.avg = (self.count*self.avg + euler[0])*1./(self.count+1)
                self.count += 1
        if self.count > 10:
            self.done = True
            self.angle_pub.publish(self.avg)
            print "Published camera angle",self.avg

if __name__=='__main__':
    rospy.init_node('delta_apriltag', anonymous=True)
    dca = deltaCameraAngle() 

    rospy.spin() 

