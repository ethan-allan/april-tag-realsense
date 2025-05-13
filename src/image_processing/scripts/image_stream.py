#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy

from cv_bridge import CvBridge, CvBridgeError


def	callback(data, bridge):
	rospy.loginfo("I recieved an image")

	try:
		cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)
	except CvBridgeError as e:
		print(e)

	

	
def listener():
	rospy.init_node('april_tag_detection', anonymous=True)

	bridge = CvBridge()
	rospy.Subscriber("camera/depth/image_rect_raw", Image ,callback, bridge)
	rospy.spin()

	
if __name__ == '__main__':
	listener()
