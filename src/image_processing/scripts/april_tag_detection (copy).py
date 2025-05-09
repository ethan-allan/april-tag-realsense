#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy
import pyapriltags as apriltag
from cv_bridge import CvBridge, CvBridgeError


def	callback(data):
	rospy.loginfo("I recieved an image")

	try:
		cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
		cv2.imshow("Image window", cv_image)
		cv2.waitkey(3)
	except CvBridgeError as e:
		print(e)
		
	cv2.imshow("Image window", cv_image)
	cv2.waitkey(3)
	
print(result)
	
def listener():
	rospy.init_node('april_tag_detection', anonymous=True)
	
	bridge = CvBridge()
	rospy.Subscriber("camera/color/image_raw", Image ,callback)
	rospy.spin()

	
if __name__ == '__main__':
	listener()
