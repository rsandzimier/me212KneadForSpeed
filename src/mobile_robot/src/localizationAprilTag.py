#!/usr/bin/python

# 2.12 Lab 3 AprilTag Navigation: use AprilTag to get current robot (X,Y,Theta) in world frame, and to navigate to target (X,Y,Theta)
# Peter Yu Sept 2016

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm

from mobile_robot.msg import WheelCmdVel
from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

rospy.init_node('apriltag_navi', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()

odom_pub = rospy.Publisher("apriltag_odom", PoseWithCovarianceStamped, queue_size=10)
rp_pub = rospy.Publisher("pose_estimate", PoseStamped, queue_size=10)


COV = 0.000025
ACOV = 0.00003 
    
def main():
    apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    
    rospy.sleep(1)
    
    rospy.spin()

## apriltag msg handling function (Need to modify for Task 2)
def apriltag_callback(data):
    # use apriltag pose detection to find where is the robot
    for detection in data.detections:
        if detection.id in [0,1,3,4,6]:   # tag id is the correct one
            poselist_tag_cam = pose2poselist(detection.pose)
            poselist_tag_base = transformPose(lr, poselist_tag_cam, 'camera', 'robot_base')
            poselist_base_tag = invPoselist(poselist_tag_base)
            poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag'+str(detection.id), 'map')
            pose = PoseWithCovarianceStamped()
            pose.pose.pose.position.x = poselist_base_map[0]
            pose.pose.pose.position.y = poselist_base_map[1]
            pose.pose.pose.position.z = poselist_base_map[2]
            pose.pose.pose.orientation.x = poselist_base_map[3]
            pose.pose.pose.orientation.y = poselist_base_map[4]
            pose.pose.pose.orientation.z = poselist_base_map[5]
            pose.pose.pose.orientation.w = poselist_base_map[6]
            pose.pose.covariance = (COV, 0, 0, 0, 0, 0,
                                0, COV, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, ACOV)
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.get_rostime()
            odom_pub.publish(pose)
            #pose_viz = PoseStamped()
            #pose_viz.pose = detection.pose
            #pose_viz.header.frame_id = "map"
            #rp_pub.publish(pose_viz)

            br.sendTransform(poselist_base_map[:3], poselist_base_map[3:],
                      rospy.get_rostime(),
                      "at_base_link", "map")

if __name__=='__main__':
    main()
    
