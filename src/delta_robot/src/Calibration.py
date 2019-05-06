#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun May  5 20:46:10 2019

@author: lneil
"""

#import rospy
import numpy as np
from math import sin,cos,tan,radians,pi,atan2,degrees
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
        self.sx = 0.0015625     #1/640
        self.sy = -0.00208333333    #1/480
        
        self.cx = 320 #center of image in pixels
        self.cy = 240   
        
        self.pixelUV = [320,240] #initialize pixel coords and orientation for testing
        self.pixelOrientation = radians(45)
        
        
        
        self.CameraAngle = radians(10)  #angle of camera lens relative to vertical
        self.z0 = 1094.541              #absolute distance of camera lens from table in millimeters
        self.z = self.z0+1              #distance of camera focus from table
        self.CameraLenstoOrigin = [-38.1 , 285.75 , 101.6] #in world frame

#fx,fy,cx,cy found from camera_calibration
# These are rough estimates
#        self.fx = 642 
#        self.fy = 644    
#        self.cameraMatrix = np.array([[self.fx,0,0],[0,self.fy,0],[self.cx,self.cy,1]]) #intrinsic characteristics of camera
        
#Rotation is 180 minus camera angle    
        self.rotationMatrix = np.array([[1,0,0], 
                                        [0,cos(pi-self.CameraAngle),-sin(pi-self.CameraAngle)],
                                        [0,sin(pi-self.CameraAngle),cos(pi-self.CameraAngle)]]) #rotation from world frame to camera frame -should just be a rotation about world x-axis
    
#        self.rotationMatrix = np.array([[1,0,0], 
#                                        [0,1,0],
#                                        [0,0,1]])
        
        self.translationVector = np.array([self.CameraLenstoOrigin[0],self.CameraLenstoOrigin[1]-sin(self.CameraAngle),self.CameraLenstoOrigin[2]-cos(self.CameraAngle)])               #translation from world frame to camera frame      

        # Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
        #rospy.Timer(rospy.Duration(1./self.rate), self.function_to_loop)

#    def CameraCalibration_cb(self,msg): # the _cb suffix stands for callback. Use this suffix on your callback functions for clarity
#        # This function is called every time another node publishes to the topic called "/arbitrary_subscribed_topic_name"
#        # msg is the message that was published to "/arbitrary_subscribed_topic_name"
#        # Do whatever needs to happen when we receive this message. 
#        # For example, set a class variable to the float array in the message and increment another class variable:
#
#        self.pixelData = msg.data
#        self.pixelUV = np.array([self.pixelData[0], self.pixelData[1]])
#        self.pixelOrientation = self.pixelData[2]        
       

    def function_to_loop(self,event): # This is the function that we set a timer for above. 
#        # Do whatever you would like to do at the rate specified by the timer. 
#        # For example: print some stuff out, and publish to a topic if a certain criteria is met
        #worldxyz_msg = Float32MultiArray()
        #worldxyz_msg.data = self.PixelXYToWorldXYZ()
        #self.CameraCalibration_pub.publish(worldxyz_msg) 
        #print np.transpose((self.cameraXYZ-self.translationVector)) 
        return [self.PixelXYToWorldXYZ(self.pixelUV),degrees(self.PixelOrientationtoWorldOrientation())]
        
    def PixelXYToWorldXYZ(self,pixel):
         pixelXY = [self.sx*(pixel[0] - self.cx) , self.sy*(pixel[1] - self.cy)] #convert from pixel space to pixel X,Y frame
         cameraXYZ = np.array([pixelXY[0]*self.z0 , self.z0*pixelXY[1]/(1-tan(self.CameraAngle)*pixelXY[1]) , self.z0/(1-(tan(self.CameraAngle)*pixelXY[1]))]) #unproject pixel X,Y
         
         worldXYZ = np.matmul(np.linalg.inv(self.rotationMatrix),np.transpose((cameraXYZ)))-self.translationVector#frame change
         return worldXYZ
     
    def PixelOrientationtoWorldOrientation(self):
        pixel1 = self.PixelXYToWorldXYZ(self.pixelUV)
        pixel2 = self.PixelXYToWorldXYZ(np.array(self.pixelUV) + 100*np.array([cos(self.pixelOrientation),sin(self.pixelOrientation)]))
        self.worldOrientation = atan2(pixel2[1]-pixel1[1],pixel2[0]-pixel1[0])        
        
        return self.worldOrientation
        
if __name__ == "__main__":
    acn = CameraCalibration() # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 
    print(acn.function_to_loop(1))
    #print(acn.PIX2CAMERAMATRIX())
    #rospy.spin() # Keeps python from exiting until the ROS node is stopped

