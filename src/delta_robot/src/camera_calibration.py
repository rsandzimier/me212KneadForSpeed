#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun May  5 20:46:10 2019

@author: lneil
"""

#import rospy
import numpy as np
from math import sin,cos,tan,radians,pi,atan2,degrees
import tf
from geometry_msgs.msg import PointStamped
import rospy
#from sensor_msgs.msg import Image
#

#from std_msgs.msg import Float32MultiArray # A standard message type that allows you to use topics that publish float arrays See http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html
# There are more standard messages (std_msgs.msg) available if needed. You can also create your own custom messages if needed.
# You can also import your own python files you write (can help organize code if it is long and you want to break it up into multiple files)

class CameraCalibration: # Class name capitalized
        # Define any constant variables for for class. To use these variables in class functions, use self.ARBITRARY_CONSTANT_INT_NAME, etc
    # Use all caps for these constants

    def __init__(self): # This function is called one time as soon as an instance of a class is created
        #self.rate = 100 #[Hz]

        # Declaring a subscriber. Parameters: topic name (string), topic data type, callback function
        # Every time another node publishes a message to "/arbitrary_subscribed_topic_name", we will call the function called self.arbitrary_topic_name_cb
        # The "/" before the topic name is important
        
        #rospy.Subscriber("/usb_cam/image_raw", Image, self.CameraCalibration_cb) This was me testing
        #rospy.Subscriber("/toppings", DetectionArray, self.CameraCalibration_cb)
        #rospy.Subscriber("/self", DetectionArray, self.CameraCalibration_cb)

        # Declaring a publisher. Parameters: topic name (string), topic data type, queue_size (use queue_size=10 as default)
        #self.CameraCalibration_pub = rospy.Publisher("/worldxyz", Float32MultiArray, queue_size=10)

        # Do any other initialization of class
        #scaling factors for  millimeters to pixel conversion
        #this is related to the 
        self.tf_listener = tf.TransformListener()
        self.tf_transformer = tf.TransformerROS(True, rospy.Duration(10.0))

        self.sx = 1/642.5793848798857     #fx
        self.sy = 1/644.8809875234275     #fy
        
        self.cx = 320 #center of image in pixels
        self.cy = 240  
        
        self.pixelUV = [320,240] #initialize pixel coords and orientation for testing
        self.pixelOrientation = radians(45)
        
        
        
        self.CameraAngle = radians(10)  #angle of camera lens relative to vertical
        self.CameraHeight = 839
        self.z0 = 839/cos(self.CameraAngle)                   #851.942927372   absolute distance of camera lens from table in millimeters
        self.CameraLenstoOrigin = [271 , 36.6 , 52] #in world frame
        #self.CameraLenstoOrigin = [-80 , 36.6 , 52]
     
#Rotation is 180 minus camera angle about x   
        self.rotationMatrixX = np.array([[1,0,0], 
                                        [0,cos(pi-self.CameraAngle),-sin(pi-self.CameraAngle)],
                                        [0,sin(pi-self.CameraAngle),cos(pi-self.CameraAngle)]]) #rotation from world frame to camera frame -should just be a rotation about world x-axis
#Rotate 90 about z 
        self.rotationMatrixZ = np.array([[0,1,0],
                                         [-1,0,0],
                                         [0,0,1]]) 
        self.rotationMatrix = np.matmul(self.rotationMatrixZ,self.rotationMatrixX)  
#        self.rotationMatrix = np.array([[1,0,0], 
#                                        [0,1,0],
#                                        [0,0,1]])
        
        self.translationVector = np.array([self.CameraLenstoOrigin[0],self.CameraLenstoOrigin[1],self.CameraLenstoOrigin[2]])               #translation from world frame to camera frame      


        # Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
        #rospy.Timer(rospy.Duration(1./self.rate), self.function_to_loop)     
        
    def PixelXYToWorldXYZ(self,pixel):
         pixelXY = [self.sx*(pixel[0] - self.cx) , -self.sy*(pixel[1] - self.cy)] #convert from pixel space to pixel X,Y frame
         self.z = self.z0/(1+(tan(self.CameraAngle)*pixelXY[1]))
         cameraXYZ = np.array([pixelXY[0]*self.z , pixelXY[1]*self.z , self.z]) #unproject pixel X,Y
         p = PointStamped()
         p.header.frame_id = "delta_camera"
         p.point.x = cameraXYZ[0]
         p.point.y = cameraXYZ[1]
         p.point.z = cameraXYZ[2]
         
         #worldXYZ = np.matmul((self.rotationMatrix),np.transpose((cameraXYZ)))-self.translationVector#frame change
         pworld = self.tf_transformer.tranformPoint("delta_robot", p)
         worldXYZ = [pworld.point.x, pworld.point.y, pworld.point.z]
         return worldXYZ
     
    def PixelOrientationtoWorldOrientation(self,pixel):
        pixel1 = self.PixelXYToWorldXYZ(pixel)
        pixel2 = self.PixelXYToWorldXYZ(np.array(pixel) + 100*np.array([cos(self.pixelOrientation),sin(self.pixelOrientation)]))
        self.worldOrientation = atan2(pixel2[1]-pixel1[1],pixel2[0]-pixel1[0])        
        
        return self.worldOrientation
