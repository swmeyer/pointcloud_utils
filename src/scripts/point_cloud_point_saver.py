#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs import point_cloud2

import struct

import sys
import os

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
import tf2_py as tf2

# tf_buffer = None
# tf_listener = None

file_name = "points"
file_type = "xyz"
folder_name = "points"
file_count = 0

file_list_name = "file_list.txt"

target_frame = "base_link"

tf_found = False


#input_path = sys.argv[1]
#output_path = sys.argv[2]
#operation = sys.argv[3]

#print("Input path: ", input_path)
#print("Output path: ", output_path)

#input_file = open(input_path, 'r')
def pointcloudCallback(msg):
	global file_count
	global file_name
	global file_type
	global folder_name
	global file_list_name
	global tf_found
	
	#if timestamp from header is not zero, use that!
	timestamp_header = msg.header.stamp.secs * 1000000000 + msg.header.stamp.nsecs
	if (timestamp_header == 0):
		time_now = rospy.get_rostime()
		timestamp = time_now.secs * 1000000000 + time_now.nsecs
		time_diff = timestamp - timestamp_header
		print "Timestamp difference: " + str(time_diff) + " ns\n"
	else:
		timestamp = timestamp_header

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
			print "bad transform!\n"
			#rospy.logwarn(ex)

		except tf2.ExtrapolationException as ex:
			print "bad transform!\n"
			#rospy.logwarn(ex)
	if not tf_found:
		print "bad transform!\n"
		return

	#transform to the map frame:
	data = do_transform_cloud(msg, pointcloudCallback.trans)

	#extract from the pointlcoud 2 data enccoding:
	#from https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_apps/src/point_cloud2.py
	fmt = point_cloud2._get_struct_fmt(data.is_bigendian, data.fields)
	width, height, point_step, row_step, data = data.width, data.height, data.point_step, data.row_step, data.data
	

	filepath = os.path.join(os.getcwd(), folder_name)
	if not os.path.exists(filepath):
		print("making directory")
		print(filepath)
		print()
		os.makedirs(filepath)
	
	#filename = os.path.join(filepath, file_name + str(file_count) + "_" + str(timestamp) + "." + file_type)
	file_name_no_type = file_name + "_" + str(file_count) + "_" + str(timestamp)
	file_name_extended = file_name_no_type + "." + file_type
	filename = os.path.join(filepath, file_name_extended)
	
	file_count = file_count + 1

	print("saving to: ")
	print(filename)

	try:
		file = open(filename, "w")
	except IOError:
		print "Could not open file: " + filename + ", aborting.\n"
		return
		
	#TODO: check file type
	#todo: check if point cloud has i or not

	with file:
		if (file_type == 'xyz'):
			file.write("x, y, z, i,\n")
		else if (file_type == 'ply'):
			file.write("ply\n")
			file.write("format ascii 1.0")
			num_points = msg.height * msg.width
			element_string = "element vertex " + str(num_points)
			file.write(element_string)
			file.write("property float x")
			file.write("property float y")
			file.write("property float z")
			file.write("property float i")

		unpack_from = struct.Struct(fmt).unpack_from
		for v in xrange(height):
			offset = row_step * v
			for u in xrange(width):
				p = unpack_from(data, offset)
				offset += point_step

				write_string = str(p[0]) + ", " + str(p[1]) + ", " + str(p[2]) + ", " + str(p[3]) + ",\n"
				
				if (file_type == "ply"):
					write_string = str(p[0]) + " " + str(p[1]) + " " + str(p[2] + " " + str(p[3])) + " \n"

				file.write(write_string)

	try:
		list_file = open(file_list_name, "a")
	except IOError:
		print "Could not open list file: " + file_list_name + ", aborting.\n"
		return

	with list_file:
		list_string = str(file_count - 1) + ", " + str(timestamp) + ", " + file_name_no_type + ",\n"
		list_file.write(list_string)

def initFileList():
	global folder_name
	global file_list_name
	global list_file

	filepath = os.path.join(os.getcwd(), folder_name)
	if not os.path.exists(filepath):
		print("making directory")
		print(filepath)
		print()
		os.makedirs(filepath)

	filename = os.path.join(filepath, file_list_name)
	

	print("saving to: ")
	print(filename)

	try:
		file = open(filename, "w")
	except IOError:
		print "Could not open file: " + filename + ", aborting."
		return

	with file:
		file.write("file_number, time(ns), file_name\n")

	file_list_name = filename


def getParams():
	global target_frame
	global file_name
	global file_type
	global file_list_name

	target_frame     = rospy.get_param('~target_frame', "base_link")
	folder_name 	 = rospy.get_param('~folder_name', "points")
	file_type 		 = rospy.get_param('~file_type', "xyz")
	file_name        = rospy.get_param('~file_name', "points")
	file_list_name   = rospy.get_param('~file_list_name', "file_list.txt")



if __name__ == '__main__':
	rospy.init_node('pc_point_saver', anonymous=True)

	getParams()
	pointcloud_topic = rospy.get_param('~cloud_topic', "/cloud")

	print "Note! All file types are printed as csv files currently"

	initFileList()
	
	#todo: save header, timestamps
	queue = 100
		
	rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloudCallback, queue_size=queue, tcp_nodelay=True)

	rospy.spin()