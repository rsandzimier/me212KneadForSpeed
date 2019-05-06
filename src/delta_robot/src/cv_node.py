#!/usr/bin/python

import rospy
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from delta_robot.msg import Detection
from delta_robot.msg import DetectionArray
import cv2
import numpy as np


class CV:
	def __init__(self):
		self.rate = 100 #[Hz]

		rospy.Subscriber("/usb_cam/image_raw", Image, self.image_cb) # Need to use rectified image instead

		self.toppings_pub = rospy.Publisher("/toppings", DetectionArray, queue_size=10)
		self.slots_pub = rospy.Publisher("/slots", DetectionArray, queue_size=10)

		self.bridge = CvBridge()
		self.image_dims = []

	def image_cb(self,msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		if len(self.image_dims) == 0:
			self.image_dims = cv_image.shape
		cv2.imshow("Image Window", cv_image)
		cv2.waitKey(3)
		# Gamma filter
		cv_image = self.adjust_gamma(cv_image, 1.0)
		# Position Segment (topping location vs pizza location)

		# HSV Filter
		pepperoni = self.hsv_filter(cv_image, [155,64,50], [175,180,125], True)
		pineapple = self.hsv_filter(cv_image, [10,30,150], [30,128,225], True)
		olive = self.hsv_filter(cv_image, [125,0,0], [175,60,75], True) # Needs work
		anchovie = self.hsv_filter(cv_image, [100,30,64], [130,220,200], True) # Needs work
		# Ham
		# Open slot

		cv2.imshow("HSV", olive)
		cv2.waitKey(3)
		# Noise reduction
		pepperoni = self.noise_reduction(pepperoni,3,1)
		pineapple = self.noise_reduction(pineapple,3,1)
		anchovie = self.noise_reduction(anchovie,3,1)
		# Olive
		# Ham
		# Open slot

		# Blob detection
		pepperoni_img_poses = self.blob_detection(pepperoni, 20, 0, display=False)
		pineapple_img_poses = self.blob_detection(pineapple, 20, 0, display=False)
		anchovie_img_poses = self.blob_detection(anchovie, 20, 0, display=False)
		# Olive
		# Ham
		# Open slot

		topping_detections = []

		for p in pepperoni_img_poses:
			topping_detections.append(self.imgPose2CartesianPose(p,0))
		#for p in olive_img_poses:
		#	topping_detections.append(self.imgPose2CartesianPose(p,1))
		#for p in ham_img_poses:
		#	topping_detections.append(self.imgPose2CartesianPose(p,2))
		for p in pineapple_img_poses:
			topping_detections.append(self.imgPose2CartesianPose(p,3))
		for p in anchovie_img_poses:
			topping_detections.append(self.imgPose2CartesianPose(p,4))

		slot_detections = []
		#for p in slot_img_poses:
		#	slot_detections.append(self.imgPose2CartesianPose(p,-1))

		toppings_msg = DetectionArray()
		toppings_msg.detections = topping_detections
		self.toppings_pub.publish(toppings_msg)

		slots_msg = DetectionArray()
		slots_msg.detections = slot_detections
		self.slots_pub.publish(slots_msg)

	def imgPose2DetectionMsg(self, img_pose, detection_type):
		# Takes in an image pose (x,y in pixels and orientation in radians) and outputs a detection message (x,y,z in mm, orientation in radians, and detection type)
		# Type: 0-pepperoni, 1- olive, 2-ham, 3-pineapple, 4-anchovie, -1-open slot
		d = Detection()
		# Populate d with data x,y,z, and orientation are in cartesian space relative to the delta robot origin/coordinate system
		d.type = detection_type
		#d.position.x = 
		#d.position.y = 
		#d.position.z = 
		#d.orientation = 
		pass
		return d

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

	def blob_detection(self, image, minArea, minDist, display=False):
		# image must be binary
		# Find contours in binary image and ignore any with area less than minArea
		im2, contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours = [c for c in contours if cv2.contourArea(c) >= minArea]
		# Fill contours and dilate to merge any that are "close enough" (as defined by minDist)
		dilated = self.fillContoursBinary(contours)
		kernel = np.ones((minDist/2,minDist/2))
		dilated = cv2.dilate(dilated,kernel,iterations = 1)
		# Find new contours after dilation
		im2, contours_dilated, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		blob_masks = []
		blob_poses = []
		for c in contours_dilated:
			# Fill each contour independently. Erode by same amount as the previous dilation. Note: This could cause a single blob to be disjointed, but this is expected behavior
			# Calculate image moments and use to calculate position of centroid and principal axis.
			mask = cv2.erode(self.fillContoursBinary([c]),kernel,iterations = 1)
			mu = cv2.moments(mask, binaryImage=True)
			blob_masks.append(mask)
			if mu['m00'] == 0: continue
			centroid = [int(mu['m10']/mu['m00']), int(mu['m01']/mu['m00'])]
			I = np.asarray([[mu['mu20'],mu['mu11']],[mu['mu11'],mu['mu02']]])
			[U,S,V] = np.linalg.svd(I)
			orientation = [math.atan2(U[1,0],U[0,0])]
			blob_poses.append(centroid + orientation)

		if display: # Display each blob with a circle at the centroid and a line indicating the principal axis
			img = np.zeros(self.image_dims[0:2]).astype('uint8')
			for m in blob_masks:
				img = cv2.bitwise_or(img, m)
			img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
			for i in range(0,len(blob_poses)):
				x = blob_poses[i][0]
				y = blob_poses[i][1]
				th = blob_poses[i][2]
				cv2.circle(img, (x,y), 5, (0,0,255),thickness=-1)
				l = 20
				p1 = (int(x + l*math.cos(th)),int(y + l*math.sin(th)))
				p2 = (int(x - l*math.cos(th)),int(y - l*math.sin(th)))
				cv2.line(img, p1, p2, (0,0,255), thickness=2)
			cv2.imshow("Blobs", img)
			cv2.waitKey(3)

		return blob_poses

	def fillContoursBinary(self, contours):
		img = np.zeros(self.image_dims).astype('uint8')
		cv2.drawContours(img, contours, -1, (255,255,255), cv2.FILLED)
		return cv2.inRange(img, (127,127,127), (255,255,255))


if __name__ == "__main__":
	rospy.init_node('arbitrary_node_name', anonymous=True)
	cv = CV()

	rospy.spin()
