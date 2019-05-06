#!/usr/bin/python

import rospy
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray


class CV:
	def __init__(self):
		self.rate = 100 #[Hz]

		self.bridge = CvBridge()
		self.image_dims = []
		self.bob = None
		self.once = False
		self.depth_bob = None

		rospy.Subscriber("/camera/rgb/image_rect_color/", Image, self.image_cb) # Need to use rectified image instead


		rospy.Subscriber("/camera/depth_registered/image", Image, self.depth_cb)

		#jank
		#rospy.Subscriber("/reasonable_bob_location", Float32MultiArray, self.bob_cb)


	def depth_cb(self, msg):
		if self.bob is None:
			return
		try:
			cv_depthimage = self.bridge.imgmsg_to_cv2(msg, "32FC1")
			cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
		except CvBridgeError as e:
			print(e)

		if not self.once:
			print np.min(cv_depthimage2[np.logical_not(np.isnan(cv_depthimage2))])
			filtered_bob = self.filter_depth(self.bob, 1.0, 0.0, cv_depthimage2)
			# self.once = True
			print np.unique(filtered_bob)
			self.depth_bob = filtered_bob
				


	def image_cb(self, msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		if len(self.image_dims) == 0:
			self.image_dims = cv_image.shape
		cv_image = self.adjust_gamma(cv_image, 1)

		if self.depth_bob is not None:
			cv2.imshow("depth_bob", self.depth_bob)
			cv2.waitKey(3)

		cv2.imshow("Image Window", cv_image)
		cv2.waitKey(3)
		# Gamma filter

		# HSV Filter
		# bow_tie = self.hsv_filter(cv_image, [160,200,0], [10,255,128], True)
		# hair = self.hsv_filter(cv_image, [160,128,64], [10,200,128], True)
		# pants = self.hsv_filter(cv_image, [120,50,20], [170,125,100], True)
		# skin = self.hsv_filter(cv_image, [150,40,128], [20,128,175], True)
		# tray = self.hsv_filter(cv_image, [128,20,100], [180,100,128], True)
		bow_tie = self.hsv_filter(cv_image, [160,200,0], [10,255,128], True)
		hair = self.hsv_filter(cv_image, [160,128,64], [10,200,128], True)
		pants = self.hsv_filter(cv_image, [120,50,20], [170,125,100], True)
		skin = self.hsv_filter(cv_image, [150,40,128], [20,128,175], True)
		tray = self.hsv_filter(cv_image, [128,20,100], [180,100,128], True)
		#glass = self.hsv_filter(cv_image, [100,0,100], [150,40,175], True) # Glass too close to white color. Maybe don't use it

		# Noise reduction
		bow_tie = self.noise_reduction(bow_tie,10,1)
		hair = self.noise_reduction(hair,5,1)
		pants = self.noise_reduction(pants,5,1)
		skin = self.noise_reduction(skin,5,1)
		tray = self.noise_reduction(tray,5,1)


		cv2.imshow("bow", bow_tie)
		cv2.waitKey(3)		
		cv2.imshow("hair", hair)
		cv2.waitKey(3)		
		cv2.imshow("pants", pants)
		cv2.waitKey(3)
		cv2.imshow("skin", skin)
		cv2.waitKey(3)
		cv2.imshow("tray", tray)
		cv2.waitKey(3)

		merged_bob = bow_tie
		for p in [hair, pants, skin, tray]:
			merged_bob = cv2.bitwise_or(p, merged_bob)

		merged_bob_reduced = self.noise_reduction(merged_bob,5,1)
		
		self.bob = merged_bob_reduced
			
		cv2.imshow("bob", merged_bob)
		cv2.waitKey(3)	

		# waiter = self.merge_blobs(bow_tie, pants, 15, 20, 120, False)
		# cv2.imshow("merged_pants", cv2.bitwise_and(cv_image,cv_image,mask=waiter))
		# cv2.waitKey(3)
		# waiter = self.merge_blobs(waiter, tray, 50, 20, 50, False)
		# cv2.imshow("merged_tray", cv2.bitwise_and(cv_image,cv_image,mask=waiter))
		# cv2.waitKey(3)
		# waiter = self.merge_blobs(waiter, skin, 50, 50, 50, False)
		# cv2.imshow("merged_skin", cv2.bitwise_and(cv_image,cv_image,mask=waiter))
		# cv2.waitKey(3)
		# waiter = self.merge_blobs(waiter, hair, 50, 50, 70, False)
		# cv2.imshow("merged_hair", cv2.bitwise_and(cv_image,cv_image,mask=waiter))
		# cv2.waitKey(3)

		# # bow_tie_centroids = self.blob_detection(bow_tie, 25, 0, display=False)
		# hair_centroids = self.blob_detection(hair, 100, 0 display=False)
		# pants_centroids = self.blob_detection(pants, 100, 0, display=False)
		# skin_centroids = self.blob_detection(skin, 100, 0, display=True)
		# tray_centroids = self.blob_detection(tray, 100, 0, display=False)



		#white_background_inv = cv2.bitwise_not(white_background)
		#white_background_inv = self.noise_reduction(white_background_inv,5,1)

		#cv2.imshow("HSV_non_white", white_background_inv)
		#cv2.waitKey(3)
		#self.blob_detection(white_background_inv, 0, 0, display=False)
		# pineapple = self.noise_reduction(pineapple,3,1)
		# anchovie = self.noise_reduction(anchovie,3,1)
		# # Olive
		# # Ham
		# # Open slot

		# # Blob detection
		# pepperoni_img_poses = self.blob_detection(pepperoni, 20, 0, display=False)
		# pineapple_img_poses = self.blob_detection(pineapple, 20, 0, display=False)
		# anchovie_img_poses = self.blob_detection(anchovie, 20, 0, display=False)
		# # Olive
		# # Ham
		# # Open slot

		# topping_detections = []

		# for p in pepperoni_img_poses:
		# 	topping_detections.append(self.imgPose2CartesianPose(p,0))
		# #for p in olive_img_poses:
		# #	topping_detections.append(self.imgPose2CartesianPose(p,1))
		# #for p in ham_img_poses:
		# #	topping_detections.append(self.imgPose2CartesianPose(p,2))
		# for p in pineapple_img_poses:
		# 	topping_detections.append(self.imgPose2CartesianPose(p,3))
		# for p in anchovie_img_poses:
		# 	topping_detections.append(self.imgPose2CartesianPose(p,4))

		# slot_detections = []
		# #for p in slot_img_poses:
		# #	slot_detections.append(self.imgPose2CartesianPose(p,-1))

		# toppings_msg = DetectionArray()
		# toppings_msg.detections = topping_detections
		# self.toppings_pub.publish(toppings_msg)

		# slots_msg = DetectionArray()
		# slots_msg.detections = slot_detections
		# self.slots_pub.publish(slots_msg)

	def filter_depth(self, image, upper_depth, lower_depth, depth_image):
		# image is a np.array
		#depth_mask is a np.array
		#upper_depth, lower_Depth, float

		# for i in range(image.shape[0]):
		# 	for j in range(iamge.shape[1]):
		# 		if image[i][j] == 1:
		# 			if depth_mask[i][j] < lower_depth and depth_mask[i][j] > upper_depth:
		# 				image[i][j] == 0
		depth_mask = np.zeros_like(depth_image)
		depth_image[(np.isnan(depth_image)==True)] = upper_depth+1
		#ixs = np.where()
		ixs = []
		print np.min(depth_image), np.max(depth_image)
		# print ((lower_depth<depth_image)&(depth_image<upper_depth))
		depth_mask[(lower_depth<depth_image)&(depth_image<upper_depth)] = 255
		print np.unique((lower_depth<depth_image)&(depth_image<upper_depth)), np.unique(image)
		depth_mask = depth_mask.astype('uint8')
		# print image
		filtered_image = cv2.bitwise_and(depth_mask, image)
		return filtered_image


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


	def merge_blobs(self, image1, image2, minArea1, minArea2, minDist, display=False):
		# image must be binary
		# Find contours in binary image and ignore any with area less than minArea
		im1, contours1, hierarchy1 = cv2.findContours(image1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		im2, contours2, hierarchy2 = cv2.findContours(image2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours1 = [c for c in contours1 if cv2.contourArea(c) >= minArea1]
		contours2 = [c for c in contours2 if cv2.contourArea(c) >= minArea2]

		kernel = np.ones((minDist/2,minDist/2))
		print len(contours1),",",len(contours2)
		merged = []
		for c1 in contours1:
			for c2 in contours2:
				dilated = cv2.bitwise_or(self.fillContoursBinary([c1]), self.fillContoursBinary([c2]))
				dilated = cv2.dilate(dilated,kernel,iterations = 1)
				im2, contours_dilated, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				if len(contours_dilated) == 1:
					merged.append(contours_dilated[0])

		print len(merged)
		merged_img = cv2.erode(self.fillContoursBinary(merged),kernel,iterations = 1)

		if display:
			cv2.imshow("Merged", merged_img)
			cv2.waitKey(3)
		return merged_img


	def fillContoursBinary(self, contours):
		img = np.zeros(self.image_dims).astype('uint8')
		cv2.drawContours(img, contours, -1, (255,255,255), cv2.FILLED)
		return cv2.inRange(img, (127,127,127), (255,255,255))


if __name__ == "__main__":
	rospy.init_node('arbitrary_node_name', anonymous=True)
	cv = CV()

	rospy.spin()
