#!/usr/bin/python

import rospy
import math
import tf
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from delta_robot.msg import Detection
from delta_robot.msg import DetectionArray
import cv2
#import cv2.cv as cv
import numpy as np

class CV:

	CAMERA_TILT = 10 #Also need to change rotation in frames.launch (in radians)

	def __init__(self):
		self.rate = 100 #[Hz]
		#self.detection_phase = 1  

		rospy.Subscriber("/usb_cam/image_raw", Image, self.image_cb) # Need to use rectified image instead
		self.toppings_pub = rospy.Publisher("/toppings", DetectionArray, queue_size=10)
		self.slots_pub = rospy.Publisher("/slots", DetectionArray, queue_size=10)

		self.bridge = CvBridge()
		self.image_dims = []


		# Camera_Calibration Parameters
		self.tf_listener = tf.TransformListener()
		rospy.sleep(0.2)

		self.tf_transformer = tf.TransformerROS(True, rospy.Duration(10.0))

		self.sx = 1/642.5793848798857     #fx
		self.sy = 1/644.8809875234275     #fy

		self.cx = 185 #center of image in pixels
		self.cy = 230  

		'''self.pixelUV = [320,240] #initialize pixel coords and orientation for testing
		self.pixelOrientation = math.radians(45)'''



		self.CameraAngle = math.radians(self.CAMERA_TILT)  #angle of camera lens relative to vertical
		self.CameraHeight = 839
		self.z0 = 839/math.cos(self.CameraAngle)                   #851.942927372   absolute distance of camera lens from table in millimeters
		self.CameraLenstoOrigin = [271 , 36.6 , 52] #in world frame

		self.rotationMatrixX = np.array([[1,0,0], 
		                                [0,math.cos(math.pi-self.CameraAngle),-math.sin(math.pi-self.CameraAngle)],
		                                [0,math.sin(math.pi-self.CameraAngle),math.cos(math.pi-self.CameraAngle)]]) #rotation from world frame to camera frame -should just be a rotation about world x-axis
		#Rotate 90 about z 
		self.rotationMatrixZ = np.array([[0,1,0],
		                                 [-1,0,0],
		                                 [0,0,1]]) 
		self.rotationMatrix = np.matmul(self.rotationMatrixZ,self.rotationMatrixX)  

		self.translationVector = np.array([self.CameraLenstoOrigin[0],self.CameraLenstoOrigin[1],self.CameraLenstoOrigin[2]])
  

	def image_cb(self,msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		cv2.imshow("Image Window", cv_image)
		cv_image = cv_image[10:390,135:560]
		if len(self.image_dims) == 0:
			self.image_dims = cv_image.shape
		
		cv2.imshow("Image Window Cropped", cv_image)
		cv2.waitKey(3)

		# Gamma filter
		cv_image = self.adjust_gamma(cv_image, 1.0)
		# Position Segment (topping location vs pizza location)

		# HSV Filter
		pepperoni = self.hsv_filter(cv_image, [150,64,40], [175,180,125], True)#All set
		pineapple = self.hsv_filter(cv_image, [15,30,130], [30,128,225], True)#All set
		olive = self.hsv_filter(cv_image, [100,47,0], [150,120,80], True) #Look good
		anchovie = self.hsv_filter(cv_image, [110,75,48], [120,220,220], True) #All set
		ham = self.hsv_filter(cv_image, [150, 128, 30], [170, 240, 255], True)#All set

		saltshaker=self.hsv_filter(cv_image, [40, 30, 50], [100, 150, 150], True)
		#Ham
		#Gray Scale Image
		#everything_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		# Open slot
		slot = self.hsv_filter(cv_image, [170,180,20],[20,255,150])

		#cv2.imshow("HSV", saltshaker)
		cv2.waitKey(3)
		# Noise reduction
		pepperoni = self.noise_reduction(pepperoni,3,1)
		pineapple = self.noise_reduction(pineapple,3,1)
		anchovie = self.noise_reduction(anchovie,3,1)
		# Olive
		olive = self.olive_blob_fill(olive, 3, 5)#Fill the olive in, erode away the pepperoni shadows
		#cv2.imshow("HSV", olive)
		# Ham
		ham = self.noise_reduction(ham, 3, 1)

		#Dough detection
		dough = self.hsv_filter(cv_image, [90,30,70], [110,220,220], True)
		presser = self.hsv_filter(cv_image, [170,180,20],[20,255,150])

		dough = self.noise_reduction(dough, 3, 1)
		#presser = self.noise_reduction(presser, 3, 1)
		cv2.imshow("Dough", dough)

		dough_img_poses = self.blob_detection(dough, 60, 0, display = True)
		presser_img_poses = self.blob_detection(presser, 40, 0, display = False)
		#saltshaker = self.noise_reduction(saltshaker ,3 ,1)
		# Open slot

		# Blob detection
		pepperoni_img_poses = self.blob_detection(pepperoni, 20, 0, display=False)
		pineapple_img_poses = self.blob_detection(pineapple, 20, 0, display=False)
		anchovie_img_poses = self.blob_detection(anchovie, 20, 0, display=False)
		olive_img_poses = self.blob_detection(olive, 20, 0, display=False)
		ham_img_poses = self.blob_detection(ham, 20, 0, display=False)
		saltshaker_img_poses = self.blob_detection(saltshaker, 20, 0, display=False)
		# Olive
		#print(pepperoni_img_poses)
		# Ham
		# Open slot
		slot0 = slot.copy()
		slot = self.blob_min_area(slot, 3600)
		slot = cv2.bitwise_and(slot, slot0)
		pizza_circles = cv2.HoughCircles(slot, cv2.HOUGH_GRADIENT, 1, 60, param2=12, minRadius = 61, maxRadius=75)
		if (pizza_circles == None):
			pizza_circles = []
		else:
			pizza_circles = pizza_circles[0]
		print(pizza_circles)

		slot_circles = cv2.HoughCircles(slot, cv2.HOUGH_GRADIENT, 1, 16, param2=10, minRadius = 10, maxRadius = 18)
		if (slot_circles == None):
			slot_circles = []
		else:
			slot_circles = slot_circles[0]
		#print(slot_circles)

		#img = np.zeros(self.image_dims[0:2]).astype('uint8')
		#img = cv2.bitwise_or(img, slot)
		#img = slot
		#img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
		'''for i in range(0,len(slot_circles)):
			x = slot_circles[i][0]
			y = slot_circles[i][1]
			#th = blob_poses[i][2]
			cv2.circle(img, (x,y), 5, (0,0,255),thickness=-1)
			#l = 20
			#p1 = (int(x + l*math.cos(th)),int(y + l*math.sin(th)))
			#p2 = (int(x - l*math.cos(th)),int(y - l*math.sin(th)))
			#cv2.line(img, p1, p2, (0,0,255), thickness=2)
		cv2.imshow("Blobs", img)'''
		cv2.waitKey(3)



		topping_detections = []

		for p in pepperoni_img_poses:
			topping_detections.append(self.imgPose2DetectionMsg(p,0))
		for p in olive_img_poses:
			topping_detections.append(self.imgPose2DetectionMsg(p,1))
		for p in ham_img_poses:
			topping_detections.append(self.imgPose2DetectionMsg(p,2))
		for p in pineapple_img_poses:
			topping_detections.append(self.imgPose2DetectionMsg(p,3))
		for p in anchovie_img_poses:
			topping_detections.append(self.imgPose2DetectionMsg(p,4))
		for p in saltshaker_img_poses:
			topping_detections.append(self.imgPose2DetectionMsg(p,5))
		for p in pizza_circles:
			topping_detections.append(self.imgPose2DetectionMsg(p,6))
		for p in dough_img_poses:
			topping_detections.append(self.imgPose2DetectionMsg(p,7))
		for p in presser_img_poses:
			topping_detections.append(self.imgPose2DetectionMsg(p,8))

		slot_detections = []
		for p in slot_circles:
			slot_detections.append(self.imgPose2DetectionMsg(p,-1))

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
		pos = self.PixelXYToWorldXYZ(img_pose[0:2])
		d.position.x = pos[0]
		d.position.y = pos[1]
		d.position.z = pos[2]
		#d.orientation = 
		#print pos
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

	def olive_blob_fill(self, image, kernel_size=1, iterations=1):
		kernel = np.ones((kernel_size,kernel_size),np.uint8)
		dilation = cv2.dilate(image, kernel, iterations = iterations)
		return cv2.erode(dilation, kernel, iterations = iterations+1)

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

	def blob_min_area(self, image, minArea):
		im2, contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours = [c for c in contours if cv2.contourArea(c) >= minArea]
		# Fill contours and dilate to merge any that are "close enough" (as defined by minDist)
		dilated = self.fillContoursBinary(contours)
		return dilated

	def fillContoursBinary(self, contours):
		img = np.zeros(self.image_dims).astype('uint8')
		cv2.drawContours(img, contours, -1, (255,255,255), cv2.FILLED)
		return cv2.inRange(img, (127,127,127), (255,255,255))

	def PixelXYToWorldXYZ(self,pixel):
		pixelXY = [self.sx*(pixel[0] - self.cx) , self.sy*(pixel[1] - self.cy)] #convert from pixel space to pixel X,Y frame
		self.z = self.z0/(1+(math.tan(self.CameraAngle)*pixelXY[1]))
		cameraXYZ = np.array([pixelXY[0]*self.z , pixelXY[1]*self.z , self.z]) #unproject pixel X,Y
		p = PointStamped()
		p.header.frame_id = "delta_camera"
		p.point.x = cameraXYZ[0]/1000.0
		p.point.y = cameraXYZ[1]/1000.0
		p.point.z = cameraXYZ[2]/1000.0

		#worldXYZ = np.matmul((self.rotationMatrix),np.transpose((cameraXYZ)))-self.translationVector#frame change
		pworld = self.tf_listener.transformPoint("gripper", p)
		#print self.tf_listener.allFramesAsString()
		worldXYZ = [pworld.point.x*1000, pworld.point.y*1000, pworld.point.z*1000]
		return worldXYZ

	def PixelOrientationtoWorldOrientation(self,pixel):
		pixel1 = self.PixelXYToWorldXYZ(pixel)
		pixel2 = self.PixelXYToWorldXYZ(np.array(pixel) + 100*np.array([math.cos(self.pixelOrientation),math.sin(self.pixelOrientation)]))
		self.worldOrientation = math.atan2(pixel2[1]-pixel1[1],pixel2[0]-pixel1[0])        

		return self.worldOrientation


if __name__ == "__main__":
	rospy.init_node('arbitrary_node_name', anonymous=True)
	cv = CV()

	rospy.spin()
