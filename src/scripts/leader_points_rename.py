#!/usr/bin/env python
import rospy

import struct

import sys
import os

# tf_buffer = None
# tf_listener = None

global filepath
file_name = "points"
file_type = "xyz"
folder_name = "points"
file_count = 0

file_list_name = "file_list.txt"

file_list = dict()


def processFile(filename, timestamp):
	global file_name
	global file_type
	global file_list

	#print file_list
	#print "\n"

	##open the old file and copy the contents
	#try:
	#	file = open(filename, "r")
	#except IOError:
	#	print "Could not open file: " + filename + ", aborting.\n"
	#	return
#
	#print "Processing file " + filename + "\n"
#
	#with file:
	#	pass #todo: copy file contents

	#TODO: get file count from dictionary
	key = str(timestamp)
	if (file_list.has_key(key)):
		file_count = file_list[key]
	else:
		print "Time " + key + " is not in the file list! Skipping file\n"
		return

	#generate new file path
	new_filepath = os.path.join(filepath, "new")
	if not os.path.exists(new_filepath):
		print("making directory")
		print(new_filepath)
		print()
		os.makedirs(new_filepath)

	file_name_extended = file_name + "_" + str(file_count) + "_" + str(timestamp) + "_leader." + file_type
	new_filename = os.path.join(new_filepath, file_name_extended)

	print "Renaming file \n    " + filename + "\n to " + new_filename + "\n"

	##open the new file and copy in the old contents
	#try:
	#	file = open(new_filename, "w")
	#except IOError:
	#	print "Could not open file: " + new_filename + ", aborting.\n"
	#	return
#
	#with file:
	#	pass #todo: copy in file content!

	#rename the file!
	os.rename(filename, new_filename)


def processDirectory():
	global filepath
	global file_list_name

	print "Scanning directory \"" + filepath + "\" for files\n"

	entries = os.listdir(filepath)
	for entry in entries:
		#print "Found file " + entry + "\n"
		tokens = entry.split("_")

		if (len(tokens) < 4):
			continue

		timestamp = tokens[3].split(".")[0]

		filename = os.path.join(filepath, entry)
		processFile(filename, timestamp)



def importFileList(file_list_name):
	global file_list

	print "Opening list file: " + file_list_name + "\n"

	try:
		file = open(file_list_name, "r")
	except IOError:
		print "Could not open file: " + file_list_name + ", aborting.\n"
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

			timestamp = timestamp.split()[0]
			#print "'" + timestamp + "'\n"

			print "adding file " + count + " at time " + timestamp + " to dictionary\n"

			file_list[str(timestamp)] = count

def init():
	global folder_name
	global filepath
	global file_list_name

	print("initializing file information")

	filepath = os.path.join(os.getcwd(), folder_name)
	if not os.path.exists(filepath):
		print "Folder " + filepath + " does not exist. aborting.\n"
		return False

	file_list_name = os.path.join(filepath, file_list_name)

	importFileList(file_list_name)

	print("Initialization finished successfully.")
	return True


def getParams():
	global target_frame
	global file_name
	global file_type
	global file_list_name
	global folder_name

	print("loading in parameters")
	folder_name 	 = rospy.get_param('~folder_name', "points")
	file_type 		 = rospy.get_param('~file_type', "xyz")
	file_name        = rospy.get_param('~file_name', "points")
	file_list_name   = rospy.get_param('~file_list_name', "file_list.txt")

	print("parameters lodaded")



if __name__ == '__main__':
	rospy.init_node('leader_file_renamer', anonymous=True)

	getParams()

	initialized = init()

	if (initialized):
		processDirectory()

	print("end")
