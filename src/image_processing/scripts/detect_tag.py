#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy
import pyapriltags as apriltag
from cv_bridge import CvBridge, CvBridgeError

# ff
def	callback(data, bridge):
	rospy.loginfo("I recieved an image")

	try:
		cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
		grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		detector = apriltag.Detector(families='tagStandard41h12')
		#detector = apriltag.Detector()
		at_result = detector.detect(grey_image)
		print(at_result)
		draw_image = cv2.cvtColor(grey_image, cv2.COLOR_GRAY2BGR)
		
		for detection in at_result:
			corners = detection.corners.astype(int)
			for i in range(4):
				pt1 = tuple(corners[i])
				pt2 = tuple(corners[(i + 1) % 4])  # Wrap around to first point
				cv2.line(draw_image, pt1, pt2, (0, 0, 255), 2)

			center = tuple(detection.center.astype(int))
			cv2.putText(draw_image, f"ID: {detection.tag_id}", center,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			
		# if video_writer is None:
		# 	height, width = draw_image.shape[:2]
		# 	frame_size = (width, height)
		# 	fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Or use 'MJPG', 'MP4V', etc.
		# 	video_writer = cv2.VideoWriter("output_vid.avi", fourcc, 10, frame_size)

		# video_writer.write(draw_image)

		cv2.imshow("Image window", draw_image)
		cv2.waitKey(1)


	except CvBridgeError as e:
		print(e)

def on_shutdown(video_writer):
    if video_writer:
        video_writer.release()
        print("Video file saved and writer released.")


	
def listener():
	rospy.init_node('april_tag_detection', anonymous=True)
	#rospy.on_shutdown(on_shutdown)
	#Start ROS-CV Bridge
	bridge = CvBridge()

	#Setup Video log
#
	video_writer = None
	video_path = "output_vid.avi"  # Change path as needed
	fps = 10  # Frames per second
	frame_size = None  # To be set on first callback

	# Setup ROS node
	rospy.Subscriber("camera/color/image_raw", Image ,callback, bridge)
	rospy.spin()

	
if __name__ == '__main__':
	listener()
