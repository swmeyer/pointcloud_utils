#!/usr/bin/env python
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import struct
import time

import sys
import os

# tf_buffer = None
# tf_listener = None

frame = "lidar"
global filepath
file_name = "points"
file_type = "xyz"
folder_name = "points"
file_count = 0
file_list_name = "file_list.txt"
point_topic = "/points"
global points_pub


def processFile(filename, timestamp):
	global file_name
	global frame

	#open the old file and copy the contents
	try:
		file = open(filename, "r")
	except IOError:
		#print "Could not open file: \'" + filename + "\', aborting.\n"
		return

	print "Processing file " + filename + "\n"

	cloud = []
	has_i = False
	with file:
		lines = file.readlines()
		skip = True

		#skip the header line (first line)
		for line in lines:
			if (skip):
				skip = False
				continue

			tokens = line.split()
			x = float(tokens[0])
			y = float(tokens[1])
			z = float(tokens[2])
			if (has_i or (len(tokens) > 3) ):
				i = float(tokens[3])
				pt = [x, y, z, i]
				has_i = True
			else:
				pt = [x, y, z]

			cloud.append(pt)

	fields = []
 	if (has_i):
 		fields = [PointField('x', 0, PointField.FLOAT32, 1),
 				  PointField('y', 0, PointField.FLOAT32, 1),
 				  PointField('z', 0, PointField.FLOAT32, 1),
 				  PointField('i', 0, PointField.FLOAT32, 1)]
	else:
		fields = [PointField('x', 0, PointField.FLOAT32, 1),
 				  PointField('y', 0, PointField.FLOAT32, 1),
 				  PointField('z', 0, PointField.FLOAT32, 1)]

 	header = Header()
	header.frame_id = frame
	#print "Time in sec: " + str(int(timestamp)/1000000000.0) + "\n"
	header.stamp = rospy.Time.from_sec(int(timestamp)/1000000000.0)
	pc2 = point_cloud2.create_cloud(header, fields, cloud)

	print "Publishing cloud at time " + str(header.stamp.secs) + "." + str(header.stamp.nsecs) + "\n"
	points_pub.publish(pc2)
	time.sleep(1)


def processDirectory():
	global filepath
	global file_list_name
	global file_type

	#print "Scanning directory \"" + filepath + "\" for files\n"

	#entries = os.listdir(filepath)
	#for entry in entries:
	#	#print "Found file " + entry + "\n"
	#	tokens = entry.split("_")
	#	timestamp = tokens[2]
#
	#	filename = os.path.join(filepath, entry)
	#	processFile(filename, timestamp)

	#get file names to import from file_list
	print "Opening list file: " + file_list_name + "\n"

	try:
		file = open(file_list_name, "r")
	except IOError:
		print "Could not open file: \'" + file_list_name + "\', aborting.\n"
		return

	print "List file opened!\n"

	#save order number and timestamp from file to a dictionary
	with file:
		lines = file.readlines()
		skip = True

		#skip the header line (first line)
		for line in lines:
			if (skip):
				skip = False
				continue

			tokens = line.split(",")
			count = tokens[0]
			timestamp = tokens[1]
			filename = tokens[2].split()[0] + "_leader." + file_type
			filename = os.path.join(filepath, filename)

			processFile(filename, timestamp)


def init():
	global folder_name
	global filepath
	global file_list_name
	global point_topic
	global points_pub

	points_pub = rospy.Publisher(point_topic, PointCloud2, queue_size=10)

	print("initializing file information")

	filepath = os.path.join(os.getcwd(), folder_name)
	if not os.path.exists(filepath):
		print "Folder " + filepath + " does not exist. aborting.\n"
		return False

	file_list_name = os.path.join(filepath, file_list_name)

	print("Initialization finished successfully.")
	return True


def getParams():
	global target_frame
	global file_name
	global file_type
	global file_list_name
	global folder_name
	global point_topic
	global frame

	print("loading in parameters")
	folder_name 	 = rospy.get_param('~folder_name', "points")
	file_type 		 = rospy.get_param('~file_type', "xyz")
	file_name        = rospy.get_param('~file_name', "points")
	file_list_name   = rospy.get_param('~file_list_name', "file_list.txt")
	point_topic 	 = rospy.get_param('~point_topic', "/leader_points")
	frame 			 = rospy.get_param('~frame', "lidar")

	print("parameters lodaded")


if __name__ == '__main__':
	rospy.init_node('point_playback', anonymous=True)

	getParams()

	initialized = init()

	if (initialized):
		processDirectory()

	print("end")
