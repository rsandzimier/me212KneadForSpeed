#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
Created on Mon Apr 29 18:23:51 2019

@author: lneil
'''

# A template to use for making a ROS node. Shows how to publish/subscribe to topics and how to structure the node as a class
# This is not the only way, but for consistency across our team try not to deviate too far so we can all have an easier time reading each other's code

# Import any necessary libraries
import rospy
import numpy as np
from math import sin,cos,tan,radians,pi
from sensor_msgs.msg import Image


from std_msgs.msg import Float32MultiArray # A standard message type that allows you to use topics that publish float arrays See http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html
# There are more standard messages (std_msgs.msg) available if needed. You can also create your own custom messages if needed.
# You can also import your own python files you write (can help organize code if it is long and you want to break it up into multiple files)

class CameraCalibration: # Class name capitalized
        # Define any constant variables for for class. To use these variables in class functions, use self.ARBITRARY_CONSTANT_INT_NAME, etc
    # Use all caps for these constants

    def __init__(self): # This function is called one time as soon as an instance of a class is created
        self.rate = 100 #[Hz]

        # Declaring a subscriber. Parameters: topic name (string), topic data type, callback function
        # Every time another node publishes a message to "/arbitrary_subscribed_topic_name", we will call the function called self.arbitrary_topic_name_cb
        # The "/" before the topic name is important
        
        #rospy.Subscriber("/usb_cam/image_raw", Image, self.CameraCalibration_cb) This was me testing
        #rospy.Subscriber("/toppings", DetectionArray, self.CameraCalibration_cb)
        #rospy.Subscriber("/self", DetectionArray, self.CameraCalibration_cb)

        # Declaring a publisher. Parameters: topic name (string), topic data type, queue_size (use queue_size=10 as default)
        self.CameraCalibration_pub = rospy.Publisher("/worldxyz", Float32MultiArray, queue_size=10)

        # Do any other initialization of class
        #scaling factors for  millimeters to pixel conversion
        #this is related to the 
        self.sx = 1/6.4     #1/6.40
        self.sy = 1/4.8     #1/4.80
        
        self.cx = 320 #center of image in pixels
        self.cy = 240   
        
        self.pixelUV = [320,240] #initialize pixel coords and orientation
        self.pixelOrientation = 0
        
        
        
        self.CameraAngle = radians(10)  #angle of camera lens relative to vertical
        self.z0 = 1123                  #absolute distance of camera lens from table in millimeters
        self.z = self.z0+1000             #distance of camera focus from table
        self.CameraLenstoOrigin = [-38.1 , 285.75 , 101.6]

#fx,fy,cx,cy found from camera_calibration
        
#        self.fx = 2 
#        self.fy = 2    
#        self.cameraMatrix = np.array([[self.fx,0,0],[0,self.fy,0],[self.cx,self.cy,1]]) #intrinsic characteristics of camera
        
    
        self.rotationMatrix = np.array([[1,0,0], 
                                        [0,cos(pi-self.CameraAngle),-sin(pi-self.CameraAngle)],
                                        [0,sin(pi-self.CameraAngle),cos(pi-self.CameraAngle)]]) #rotation from world frame to camera frame
        self.translationVector = np.array([self.CameraLenstoOrigin[0],self.CameraLenstoOrigin[1]-sin(self.CameraAngle),self.CameraLenstoOrigin[2]-cos(self.CameraAngle)])               #translation from world frame to camera frame      

        # Set up timers. Parameters: t (time in seconds), function. Will call the specified function every t seconds until timer is killed or node is killed 
        rospy.Timer(rospy.Duration(1./self.rate), self.function_to_loop)

    def CameraCalibration_cb(self,msg): # the _cb suffix stands for callback. Use this suffix on your callback functions for clarity
        # This function is called every time another node publishes to the topic called "/arbitrary_subscribed_topic_name"
        # msg is the message that was published to "/arbitrary_subscribed_topic_name"
        # Do whatever needs to happen when we receive this message. 
        # For example, set a class variable to the float array in the message and increment another class variable:

        self.pixelData = msg.data
        self.pixelUV = np.array([self.pixelData[0], self.pixelData[1]])
        self.pixelOrientation = self.pixelData[2]        
       

    def function_to_loop(self,event): # This is the function that we set a timer for above. 
#        # Do whatever you would like to do at the rate specified by the timer. 
#        # For example: print some stuff out, and publish to a topic if a certain criteria is met
        worldxyz_msg = Float32MultiArray()
        worldxyz_msg.data = self.PixelXYToWorldXYZ()
        self.CameraCalibration_pub.publish(worldxyz_msg) 
        #print np.transpose((self.cameraXYZ-self.translationVector)) 
        print self.worldXYZ
        
    def PixelXYToWorldXYZ(self):
         self.pixelXY = [self.sx*(self.pixelUV[0] - self.cx) , self.sy*(self.pixelUV[1] - self.cy)]
         self.cameraXYZ = np.array([self.pixelXY[0]*self.z0 , self.z0*self.pixelXY[1]/(1-tan(self.CameraAngle)*self.pixelXY[1]) , self.z0/(1-tan(self.CameraAngle)*self.pixelXY[1])])
         
         self.worldXYZ = np.matmul(np.linalg.inv(self.rotationMatrix),np.transpose((self.cameraXYZ-self.translationVector)))    
         return self.worldXYZ
     
    def PixelOrientationtoWorldOrientation(self):
        self.pixel1 = self.pixelUV
        self.pixel2 = self.pixelUV + 
        
        
        return self.worldOrientation


if __name__ == "__main__":
    rospy.init_node('camera_calibration', anonymous=True) # Initialize the node
    acn = CameraCalibration() # Create an instance of the class (which will handle all the publishing , subscribing, etc. See above). 
    
    rospy.spin() # Keeps python from exiting until the ROS node is stopped

