#!/usr/bin/python

# 2.12 Lab 3 me212bot_node: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016

import rospy
import threading
import serial
import tf
import tf.transformations as tfm
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry

import helper
from mobile_robot.msg import WheelCmdVel

COV = .0000000025 #(m^2)
ACOV = 0.00000025

EPSILON = .000001

serialComm = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)
broadcaster = tf.TransformBroadcaster()
odom_pub = rospy.Publisher("/odom", PoseWithCovarianceStamped, queue_size=10)

## main function (Need to modify)
def main():
    #rospy.init_node('me212bot', anonymous=True)
    rospy.init_node('mobile_robot', anonymous=True)
    
    odometry_thread = threading.Thread(target = read_odometry_loop)
    odometry_thread.start()
    
    ## 1. Initialize a subscriber
    rospy.Subscriber('/cmdvel', WheelCmdVel, cmdvel_callback)
    
    rospy.spin()


## msg handling function (Need to modify)
def cmdvel_callback(msg):  
    ## 2. Send msg.desiredWV_R and msg.desiredWV_L to Arduino.
    strCmd =  str(msg.desiredWV_R) + ',' + str(msg.desiredWV_L) + '\n'
    serialComm.write(strCmd)
    


# read_odometry_loop() is for reading odometry from Arduino and publish to rostopic. (No need to modify)
def read_odometry_loop():
    prev_x, prev_y, prev_theta = 0, 0, 0
    prevtime = rospy.Time.now()
    while not rospy.is_shutdown():
        # get a line of string that represent current odometry from serial
        serialData = serialComm.readline()
        
        # split the string e.g. "0.1,0.2,0.1" with cammas
        splitData = serialData.split(',')
        
        # parse the 3 split strings into 3 floats

        x, y, theta = 0, 0, 0

        try:
            x     = float(splitData[0])
            y     = float(splitData[1])
            theta = float(splitData[2])
            
            hz    = 1.0 / (rospy.Time.now().to_sec() - prevtime.to_sec())
            prevtime = rospy.Time.now()
            
            print 'x=', x, ' y=', y, ' theta =', theta, ' hz =', hz

            
        except:
            # print out msg if there is an error parsing a serial msg
            print 'Cannot parse', splitData
            
        # publish odometry as Pose msg
        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y

        qtuple = tfm.quaternion_from_euler(0, 0, theta)
        qx, qy, qz, qw = qtuple
        pose.pose.pose.orientation = Quaternion(qx, qy, qz, qw)

        if abs(prev_x - x) < EPSILON and abs(prev_y - y) < EPSILON and abs(prev_theta - theta) < EPSILON:
            pose.pose.covariance = (0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0)
        else:
            pose.pose.covariance = (COV, 0, 0, 0, 0, 0,
                                    0, COV, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, ACOV)

        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.get_rostime()
        odom_pub.publish(pose)
                                
        broadcaster.sendTransform((x, y, 0.0), qtuple,
                              prevtime,
                              "base_link", "odom")

        prev_x, prev_y, prev_theta = x, y, theta

            

if __name__=='__main__':
    main()


