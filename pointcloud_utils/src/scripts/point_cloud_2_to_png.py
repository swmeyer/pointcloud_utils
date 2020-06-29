#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs import point_cloud2

import struct

import cv2
from cv_bridge import CvBridge
import numpy as np

import sys
import os

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
import tf2_py as tf2

# tf_buffer = None
# tf_listener = None

bridge = CvBridge()

show_image = False
publish_image = False
save_image = True
binary_mode = True


map_height = 1024
map_width = 1024
resolution = 0.1

z_scale_max = -3
z_scale_min = 3

x_min = -10
x_max = 200
y_min = -50
y_max = 50
z_min = 0
z_max = 100

file_count = 0

target_frame = "base_link"


tf_found = False
centered = False
centered_y = True

img_pub = rospy.Publisher("/pc_image_BOV", Image, queue_size=1)

#input_path = sys.argv[1]
#output_path = sys.argv[2]
#operation = sys.argv[3]

#print("Input path: ", input_path)
#print("Output path: ", output_path)

#input_file = open(input_path, 'r')
def pointcloudCallback(msg):
	# print(data.header.stamp)
	global tf_found
	#find the tf once:
	if not tf_found:
		tf_buffer = tf2_ros.Buffer()
		tf_listener = tf2_ros.TransformListener(tf_buffer)

	while (not tf_found and not rospy.is_shutdown()):
		try:
			pointcloudCallback.trans = tf_buffer.lookup_transform(target_frame, msg.header.frame_id,
    	                                       rospy.get_rostime())
			tf_found = True
		except tf2.LookupException as ex:
			pass
			#rospy.logwarn(ex)

		except tf2.ExtrapolationException as ex:
			pass
			#rospy.logwarn(ex)
	if not tf_found:
		return

	#transform to the map frame:
	data = do_transform_cloud(msg, pointcloudCallback.trans)

	#extract from the pointlcoud 2 data enccoding:
	#from https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_apps/src/point_cloud2.py
	fmt = point_cloud2._get_struct_fmt(data.is_bigendian, data.fields)
	width, height, point_step, row_step, data = data.width, data.height, data.point_step, data.row_step, data.data
	
	#sample into image grid:
	img = np.zeros((map_height, map_width), dtype = "uint8")
	img_h = np.zeros((map_height, map_width), dtype = "uint8")

	unpack_from = struct.Struct(fmt).unpack_from
	for v in xrange(height):
		offset = row_step * v
		for u in xrange(width):
			p = unpack_from(data, offset)
			offset += point_step
			if (p[2] < z_min or p[2] > z_max):
				continue

			global centered
			if centered:
				i = int( (-p[0] ) / resolution + ( map_height / 2))
				j = int( (-p[1] ) / resolution + ( map_width / 2) )
			elif centered_y:
				i = int( (1 - (p[0] - x_min) / (map_height * resolution)) * map_height )
				j = int( (-p[1] ) / resolution + ( map_width / 2)					   )
			else:
				i = int( (1 - (p[0] - x_min) / (map_height * resolution)) * map_height )
				j = int( (1 - (p[1] - y_min) / (map_width * resolution)) * map_width   )

			#if point out of bounds, discard
			if (i < map_height and j < map_width and i >= 0 and j >= 0):
				#increase grid count
				if binary_mode:
					img[i,j] = 255
				else: 
					img_h[i,j] = max(img_h[i,j], p[2]) #real-valued height grid
					img[i,j] = (img_h[i,j] * 256) / (z_scale_max - z_scale_min) #scaled value height grid //TODO: disallow negatives!?
	
	#publish image:
	if publish_image:
		#NOTE: extreme lag when running realtime
		img_msg = bridge.cv2_to_imgmsg(img, encoding="mono8") 
		# img_msg = Image()
		# img_msg.header = msg.header
		# img_msg.height = map_height
		# img_msg.width = map_width
		# img_msg.step = map_width #one byte encoding

		# img_msg.encoding = "mono8"
		# img_msg.data = img
	
		img_pub.publish(img_msg);

	if save_image:
		global file_count
		filepath = os.path.join(os.getcwd(), "images")
		if not os.path.exists(filepath):
			print("making directory")
			print(filepath)
			print()
			os.makedirs(filepath)
		print("saving to: ")
		print(filepath)
		filename = os.path.join(filepath, "points" + str(file_count) + ".png")
		
		
		file_count = file_count + 1
	
		global show_image
		if show_image:
			cv2.imshow(filename, img)
			cv2.waitKey(0)
			cv2.destroyAllWindows()
	
		cv2.imwrite( filename, img )

if __name__ == '__main__':
	rospy.init_node('pc_saver', anonymous=True)

	pointcloud_topic = rospy.get_param('~cloud_topic', "/cloud")
	target_frame     = rospy.get_param('~target_frame', "base_link")
	binary_mode      = rospy.get_param('~binary_mode', False)
	save_image       = rospy.get_param('~save_image', False)
	publish_image 	 = rospy.get_param('~publish_iamge', True)

	#todo: get map params here
	#todo: get save dir here

	#todo: save header, timestamps
	queue = 1
	if save_image:
		queue = 10
		
	rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloudCallback, queue_size=queue)

	rospy.spin()