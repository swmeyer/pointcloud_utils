#!/usr/bin/env python
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from decimal import Decimal

import struct
import time

import sys
import os

import threading

import rosbag


# tf_buffer = None
# tf_listener = None

frame = "lidar"
global filepath
file_type = "xyz"
folder_path = "points"
file_count = 0
file_list_name = "file_list.txt"
point_topic = "/points"
pointcloud_sync_topic = "/cloud"
global points_pub
global current_timestamp
global current_cloud
global has_intensity
global time_is_in_seconds
published = False
global save_to_bagfile
global publish_points
global bagfile

def pointcloudCallback(msg):
	global current_timestamp
	global points_pub
	global current_cloud
	global published

	#TODO: handle the case where pointcloud has invalid timestamp

	#print "time delay: " + str(current_timestamp - msg.header.stamp) + "\n"

	#if (current_timestamp <= msg.header.stamp):
	#	points_pub.publish(current_cloud)
	#	published = True
	
	points_pub.publish(current_cloud)
	published = True

def writeToBag(msg):
	global bagfile
	global point_topic

	bag = rosbag.Bag(bagfile, 'a')

	try:
		bag.write(point_topic, msg)
	finally:
		bag.close()


def processFile(filename, timestamp):
	global frame
	global current_cloud
	global current_timestamp
	global published
	global save_to_bagfile
	global publish_points
	global has_intensity
	global time_is_in_seconds
	global use_live_cloud_sync

	#open the old file and copy the contents
	try:
		file = open(filename, "r")
	except IOError:
		print "Could not open file: \'" + filename + "\', printing empty cloud.\n"
		cloud = []

		fields = []
 		fields = [PointField('x', 0, PointField.FLOAT32, 1),
 				  PointField('y', 4, PointField.FLOAT32, 1),
 				  PointField('z', 8, PointField.FLOAT32, 1),
 				  PointField('intensity', 12, PointField.FLOAT32, 1)]
		
		header = Header()
		header.frame_id = frame

		if (time_is_in_seconds):
			#print "Time in sec: " + str(Decimal(timestamp)) + "\n"
			header.stamp = rospy.Time.from_sec(Decimal(timestamp))
		else:
			#print "Time in sec: " + str(Decimal(timestamp)/1000000000.0) + "\n"
			header.stamp = rospy.Time.from_sec(Decimal(timestamp)/1000000000.0)
		
		pc2 = point_cloud2.create_cloud(header, fields, cloud)
		published = False
		current_cloud = pc2
		current_timestamp = header.stamp

		if (publish_points):
			if (use_live_cloud_sync):
				while(published == False and not rospy.is_shutdown()):
					continue #wait for the cloud to be published
	
			#print "Publishing cloud at time " + str(header.stamp.secs) + "." + str(header.stamp.nsecs) + "\n"
			points_pub.publish(pc2)
			#time.sleep(1)
	
		if (save_to_bagfile):
			#Write to bag!!
			writeToBag(pc2)
		return

	print "Processing file " + filename + "\n"

	cloud = []
	with file:
		lines = file.readlines()
		skip = True

		#skip the header line (first line)
		for line in lines:
			if (skip):
				skip = False
				continue

			if (file_type == "xyz"):
				tokens = line.split()
			elif (file_type == "csv"):
				tokens = line.split(",")
			else:
				print "ERROR! File type not supported" + file_type + "\n"
				return

			x = float(tokens[0])
			y = float(tokens[1])
			z = float(tokens[2])
			if (has_intensity):
				i = float(tokens[3])
				pt = [x, y, z, i]
				has_intensity = True
			else:
				pt = [x, y, z]

			cloud.append(pt)

	fields = []
 	if (has_intensity):
 		fields = [PointField('x', 0, PointField.FLOAT32, 1),
 				  PointField('y', 4, PointField.FLOAT32, 1),
 				  PointField('z', 8, PointField.FLOAT32, 1),
 				  PointField('intensity', 12, PointField.FLOAT32, 1)]
	else:
		fields = [PointField('x', 0, PointField.FLOAT32, 1),
 				  PointField('y', 4, PointField.FLOAT32, 1),
 				  PointField('z', 8, PointField.FLOAT32, 1)]

 	header = Header()
	header.frame_id = frame
	if (time_is_in_seconds):
		#print "Time in sec: " + str(Decimal(timestamp)) + "\n"
		header.stamp = rospy.Time.from_sec(Decimal(timestamp))
	else:
		#print "Time in sec: " + str(Decimal(timestamp)/1000000000.0) + "\n"
		header.stamp = rospy.Time.from_sec(Decimal(timestamp)/1000000000.0)

	pc2 = point_cloud2.create_cloud(header, fields, cloud)
	published = False
	current_cloud = pc2
	current_timestamp = header.stamp

	if (publish_points):
		if (use_live_cloud_sync):
			while(published == False and not rospy.is_shutdown()):
				continue #wait for the cloud to be published

		#print "Publishing cloud at time " + str(header.stamp.secs) + "." + str(header.stamp.nsecs) + "\n"
		points_pub.publish(pc2)
		#time.sleep(1)

	if (save_to_bagfile):
		#Write to bag!!
		writeToBag(pc2)


def processDirectory():
	global filepath
	global file_list_name
	global file_type
	global save_to_bagfile
	global publish_points
	global frame

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

			if (rospy.is_shutdown()):
				return

			tokens = line.split(",")
			count = tokens[0]
			timestamp = tokens[1]
			filename = tokens[2].split()[0] + "." + file_type
			filename = os.path.join(filepath, filename)

			processFile(filename, timestamp)


def init():
	global folder_path
	global filepath
	global file_list_name
	global point_topic
	global points_pub
	global pointcloud_sync_topic

	points_pub = rospy.Publisher(point_topic, PointCloud2, queue_size=10)

	print("initializing file information")

	#filepath = os.path.join(os.getcwd(), folder_path)
	filepath = folder_path
	if not os.path.exists(filepath):
		print "Folder " + filepath + " does not exist. aborting.\n"
		return False

	file_list_name = os.path.join(filepath, file_list_name)

	queue = 1
	rospy.Subscription(pointcloud_sync_topic, PointCloud2, pointcloudCallback, queue_size=queue)

	print("Initialization finished successfully.")
	return True


def getParams():
	global file_type
	global file_list_name
	global folder_path
	global point_topic
	global frame
	global bagfile
	global save_to_bagfile
	global publish_points
	global pointcloud_sync_topic
	global has_intensity
	global time_is_in_seconds
	global use_live_cloud_sync

	print("loading in parameters")
	folder_path 	 = rospy.get_param('~folder_path', "points")
	file_type 		 = rospy.get_param('~file_type', "xyz")
	file_list_name   = rospy.get_param('~file_list_name', "file_list.txt")
	point_topic 	 = rospy.get_param('~point_topic', "/leader_points")
	frame 			 = rospy.get_param('~frame', "lidar")
	bagfile 		 = rospy.get_param('~bagfile_name', "points.bag")
	save_to_bagfile  = rospy.get_param('~save_to_bagfile', False)
	publish_points   = rospy.get_param('~publish_points', True) 
	has_intensity    = rospy.get_param('~has_intensity', False)
	time_is_in_seconds = rospy.get_param('~time_is_in_seconds', False)
	use_live_cloud_sync = rospy.get_param('~use_live_cloud_sync', False)

	pointcloud_sync_topic = rospy.get_param('~cloud_sync_topic', "/cloud")

	if (save_to_bagfile):
		print "Bagfile: " + bagfile + "\n"

		bag = rosbag.Bag(bagfile, 'w')
		bag.close()

	print("parameters lodaded")


if __name__ == '__main__':
	rospy.init_node('point_playback', anonymous=True)

	getParams()

	initialized = init()

	if (initialized):
		process_thread = threading.Thread(target=processDirectory)
		process_thread.start()
		#processDirectory()

	rospy.spin()

	print("end")
