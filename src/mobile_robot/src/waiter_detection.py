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
		self.image_dims = None
		self.bob = None
		self.once = False

		self.rgb_image = None
		self.depth_image = None

		rospy.Subscriber("/camera/rgb/image_rect_color/", Image, self.image_cb) # Need to use rectified image instead

		rospy.Subscriber("/camera/depth_registered/image", Image, self.depth_cb)

		#jank
		#rospy.Subscriber("/reasonable_bob_location", Float32MultiArray, self.bob_cb)


	def depth_cb(self, msg):
		try:
			cv_depthimage = self.bridge.imgmsg_to_cv2(msg, "32FC1")
			self.depth_image = np.array(cv_depthimage, dtype=np.float32)
		except CvBridgeError as e:
			print(e)

	def image_cb(self, msg):
		try:
			self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			if self.image_dims is None:
				self.image_dims = self.rgb_image.shape
		except CvBridgeError as e:
			print(e)
		# cv2.imshow("Image Window", self.rgb_image)
		# cv2.waitKey(3)
		self.process()

	def process(self):
		if self.rgb_image == None or self.depth_image == None: 
			return
		depth_mask = self.filter_depth(self.depth_image, 1.0, 0.0)

		cv_image = cv2.bitwise_and(self.rgb_image,self.rgb_image,mask = depth_mask)

		cv_image = self.adjust_gamma(cv_image, 1)

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
		bow_tie = self.noise_reduction(bow_tie,3,1)
		hair = self.noise_reduction(hair,3,1)
		pants = self.noise_reduction(pants,3,1)
		#skin = self.noise_reduction(skin,1,1)
		#tray = self.noise_reduction(tray,1,1)


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
		for p in [hair, pants]:
			merged_bob = cv2.bitwise_or(p, merged_bob)

		merged_bob_reduced = self.noise_reduction(merged_bob,3,1)
		
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
		self.rgb_image = None
		self.depth_image = None

	def filter_depth(self, depth_image, upper_depth, lower_depth):
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
		# print ((lower_depth<depth_image)&(depth_image<upper_depth))
		depth_mask[(lower_depth<depth_image)&(depth_image<upper_depth)] = 255
		depth_mask = depth_mask.astype('uint8')
		return depth_mask

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


	def fillContoursBinary(self, contours):
		img = np.zeros(self.image_dims).astype('uint8')
		cv2.drawContours(img, contours, -1, (255,255,255), cv2.FILLED)
		return cv2.inRange(img, (127,127,127), (255,255,255))


if __name__ == "__main__":
	rospy.init_node('arbitrary_node_name', anonymous=True)
	cv = CV()

	rospy.spin()
