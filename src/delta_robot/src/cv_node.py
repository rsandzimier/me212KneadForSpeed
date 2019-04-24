#!/usr/bin/python

import rospy
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CV:
	def __init__(self): # This function is called one time as soon as an instance of a class is created
		self.rate = 100 #[Hz]

		rospy.Subscriber("/usb_cam/image_raw", Image, self.image_cb)

		self.bridge = CvBridge()

		# Set up publishers for toppings and holes
		#self.toppings_pub = rospy.Publisher("/toppings", , queue_size=10)
		#self.holes_pub = rospy.Publisher("/holes", , queue_size=10)

	def image_cb(self,msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		except CvBridgeError as e:
			print(e)

		# Gamma filter
		cv_image = self.adjust_gamma(cv_image, 1.0)
		# Position Segment (topping location vs pizza location)
		# HSV Filter
		pepperoni = self.hsv_filter(cv_image, [0,130,125], [15,200,255], True)
		cv2.imshow("Pepperoni", pepperoni)
		cv2.waitKey(3)
		# Noise reduction
		pepperoni = self.noise_reduction(pepperoni,3,1)
		# Blob detection
		self.blob_detection(pepperoni,50,100)
		# Find centroid/orientations

		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)


	def adjust_gamma(self, image, gamma=1.0):
	   table = np.array([((i / 255.0) ** (1.0 / gamma)) * 255 for i in np.arange(0, 256)]).astype("uint8")
	   return cv2.LUT(image, table)

	def hsv_filter(self, image, lower_hsv, upper_hsv, mask_only=True):
		lower_hsv = np.array(lower_hsv)
		upper_hsv = np.array(upper_hsv)
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		if lower_hsv[0] <= upper_hsv[0]: 
			mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
		else: # Handle case where desired hue range is at the "ends" (ie includes 0 and 255, but not hues in the middle)
			lower_hsv_temp = np.array(lower_hsv)
			lower_hsv_temp[0] = 0
			upper_hsv_temp = np.array(upper_hsv)
			upper_hsv_temp[0] = 255

			mask = cv2.bitwise_or(cv2.inRange(hsv, lower_hsv_temp, upper_hsv), cv2.inRange(hsv, lower_hsv, upper_hsv_temp))

		return mask if mask_only else cv2.bitwise_and(image,image, mask=mask)

	def noise_reduction(self, image, kernel_size=1, iterations=1):
		kernel = np.ones((kernel_size,kernel_size),np.uint8)
		erosion = cv2.erode(image,kernel,iterations = iterations)
		return cv2.dilate(erosion,kernel,iterations = iterations)

	def blob_detection(self, image, minArea, minDist):
		image = cv2.bitwise_not(image)
		params = cv2.SimpleBlobDetector_Params()
		params.filterByArea = True
		params.filterByCircularity = False;
		params.filterByConvexity = False;
		params.filterByInertia = False;
		params.minArea = minArea
		params.minThreshold = 0
		params.minDistBetweenBlobs = minDist
		detector = cv2.SimpleBlobDetector_create(params)
		keypoints = detector.detect(image)
		# Draw detected blobs as red circles.
		# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
		im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		 
		# Show keypoints
		cv2.imshow("Keypoints", im_with_keypoints)
		cv2.waitKey(0)

if __name__ == "__main__":
	rospy.init_node('arbitrary_node_name', anonymous=True)
	cv = CV()

	rospy.spin()
