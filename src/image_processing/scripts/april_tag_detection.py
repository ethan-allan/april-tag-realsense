#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

def	callback(data):
	rospy.loginfo("I recieved an image")
	
def listener():
	rospy.init_node('april_tag_detection', anonymous=True)
	rospy.Subscriber("camera/color/image_raw", Image ,callback)

	rospy.spin()

	
if __name__ == '__main__':
	listener()
