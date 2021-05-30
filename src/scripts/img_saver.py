#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import rosbag


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

def saveImageToFile(msg):
	global bridge
	global folder_name

	img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
	global file_count
	filepath = os.path.join(os.getcwd(), folder_name)
	if not os.path.exists(filepath):
		print("making directory")
		print(filepath)
		print()
		os.makedirs(filepath)
	print("saving to: ")
	print(filepath)
	filename = os.path.join(filepath, "image" + str(file_count) + "_" + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".png")

	file_count = file_count + 1
	
	cv2.imwrite( filename, img )


def imageCallback(msg):
	saveImageToFile(msg)


if __name__ == '__main__':
	print("Node starting")
	rospy.init_node('pc_saver', anonymous=True)

	img_topic = rospy.get_param('~img_topic', "/img")
	folder_name 	 = rospy.get_param('~folder_name', "images")

	bagname = rospy.get_param('~bagfile_path', "./bag.bag")
	from_bag = rospy.get_param('~from_bag', False)

	print(from_bag)

	if (from_bag):
		print("using bag", bagname)
		print("using topic", img_topic)

		bag = rosbag.Bag(bagname)
		for topic, msg, t in bag.read_messages(topics=[img_topic]):
			saveImageToFile(msg)
			if rospy.is_shutdown():
				break

		bag.close()
	else:

		#todo: save header, timestamps
		queue = 10
		
		rospy.Subscriber(img_topic, Image, imageCallback, queue_size=queue)

		rospy.spin()