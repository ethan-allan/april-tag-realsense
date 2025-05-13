#!/usr/bin/env python


import rospy
import message_filters
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
import pyapriltags as apriltag
import cv2


if (not hasattr(rs2, 'intrinsics')):
	import pyrealsense2.pyrealsense2 as rs2

class ImageListener:
	def __init__(self, depth_image_topic, depth_info_topic, colour_topic):
		self.bridge = CvBridge()

		self.sub = message_filters.Subscriber(depth_image_topic, msg_Image)
		self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)
		self.sub_col = message_filters.Subscriber(colour_topic, msg_Image)
		confidence_topic = depth_image_topic.replace('depth', 'confidence')
		self.sub_conf = rospy.Subscriber(confidence_topic, msg_Image, self.confidenceCallback)

		ats = message_filters.ApproximateTimeSynchronizer([self.sub,self.sub_col], 10, 0.1) 
		ats.registerCallback(self.imageDepthCallback)

		self.intrinsics = None
		self.pix = None
		self.pix_grade = None

	def imageDepthCallback(self, depth_img, colour_img):
		try:
			cv_col_img = self.bridge.imgmsg_to_cv2(colour_img, colour_img.encoding)
			cv_dep_img = self.bridge.imgmsg_to_cv2(depth_img, depth_img.encoding)
			center = [0,0]

			# april tag detection code
			grey_img = cv2.cvtColor(cv_col_img, cv2.COLOR_BGR2GRAY)
			detector = apriltag.Detector(families='tagStandard41h12')
			at_result = detector.detect(grey_img)

			# Loop through detected tags to get the center (should only be one tag in ws)
			for detection in at_result:
				center = tuple(detection.center.astype(int))
				#print(center)




			#pick one pixel among all the pixels with the closest range:
			indices = np.array(np.where(cv_dep_img == cv_dep_img[cv_dep_img > 0].min()))[:,0]
			pix = (indices[1], indices[0])
			self.pix = pix
			line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (center[0], center[1], cv_dep_img[center[1], center[0]])

			if self.intrinsics:
				depth = cv_dep_img[center[1], center[0]]
				result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [center[0], center[1]], depth)
				line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
			if (not self.pix_grade is None):
				line += ' Grade: %2d' % self.pix_grade
			line += '\r'
			sys.stdout.write(line)
			sys.stdout.flush()

		except CvBridgeError as e:
			print(e)
			return
		except ValueError as e:
			return
		
		# Visualize the detected tags
		#self.visualize(grey_img, at_result)



	def visualize(self, grey_img, at_result):
		draw_image = cv2.cvtColor(grey_img, cv2.COLOR_GRAY2BGR)

		# Loop through detected tags to get the center (should only be one tag in ws)
		for detection in at_result:
			center = tuple(detection.center.astype(int))
			#print(center)
			corners = detection.corners.astype(int)

			for i in range(4):
				pt1 = tuple(corners[i])
				pt2 = tuple(corners[(i + 1) % 4])
				cv2.line(draw_image, pt1, pt2, (0, 0, 255), 2)

		cv2.imshow("Image window", draw_image)
		cv2.waitKey(3)



	def confidenceCallback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
			grades = np.bitwise_and(cv_image >> 4, 0x0f)
			if (self.pix):
				self.pix_grade = grades[self.pix[1], self.pix[0]]
		except CvBridgeError as e:
			print(e)
			return



	def imageDepthInfoCallback(self, cameraInfo):
		try:
			if self.intrinsics:
				return
			self.intrinsics = rs2.intrinsics()
			self.intrinsics.width = cameraInfo.width
			self.intrinsics.height = cameraInfo.height
			self.intrinsics.ppx = cameraInfo.K[2]
			self.intrinsics.ppy = cameraInfo.K[5]
			self.intrinsics.fx = cameraInfo.K[0]
			self.intrinsics.fy = cameraInfo.K[4]
			if cameraInfo.distortion_model == 'plumb_bob':
				self.intrinsics.model = rs2.distortion.brown_conrady
			elif cameraInfo.distortion_model == 'equidistant':
				self.intrinsics.model = rs2.distortion.kannala_brandt4
			self.intrinsics.coeffs = [i for i in cameraInfo.D]
		except CvBridgeError as e:
			print(e)
			return

def main():
	depth_image_topic = '/camera/depth/image_rect_raw'
	depth_info_topic = '/camera/depth/camera_info'
	colour_image_topic = '/camera/color/image_raw'
	print ('')
	print ('show_center_depth.py')
	print ('--------------------')
	print ('App to demontrate the usage of the /camera/depth topics.')
	print ('')
	print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
	print ('Application then calculates and print the range to the closest object.')
	print ('If intrinsics data is available, it also prints the 3D location of the object')
	print ('If a confedence map is also available in the topic %s, it also prints the confidence grade.' % depth_image_topic.replace('depth', 'confidence'))
	print ('')
	
	listener = ImageListener(depth_image_topic, depth_info_topic, colour_image_topic)
	rospy.spin()

if __name__ == '__main__':
	node_name = os.path.basename(sys.argv[0]).split('.')[0]
	rospy.init_node(node_name)
	main()