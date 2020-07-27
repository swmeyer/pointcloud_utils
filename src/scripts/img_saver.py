#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image


import cv2
from cv_bridge import CvBridge
import numpy as np

import sys
import os


# tf_buffer = None
# tf_listener = None

bridge = CvBridge()

folder_name = "images"
file_count = 0


#img_pub = rospy.Publisher("/pc_image_BOV", Image, queue_size=1)

#input_path = sys.argv[1]
#output_path = sys.argv[2]
#operation = sys.argv[3]

#print("Input path: ", input_path)
#print("Output path: ", output_path)

#input_file = open(input_path, 'r')
def imageCallback(msg):
	global bridge

	img = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
	global file_count
	filepath = os.path.join(os.getcwd(), folder_name)
	if not os.path.exists(filepath):
		print("making directory")
		print(filepath)
		print()
		os.makedirs(filepath)
	print("saving to: ")
	print(filepath)
	filename = os.path.join(filepath, "points" + str(file_count) + "_" + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".png")
		
		
	file_count = file_count + 1
	
	cv2.imwrite( filename, img )

if __name__ == '__main__':
	rospy.init_node('pc_saver', anonymous=True)

	pointcloud_topic = rospy.get_param('~img_topic', "/img")
	folder_name 	 = rospy.get_param('~folder_name', "images")

	#todo: get save dir here

	#todo: save header, timestamps
	queue = 1
	if save_image:
		queue = 10
		
	rospy.Subscriber(image_topic, Image, imageCallback, queue_size=queue)

	rospy.spin()