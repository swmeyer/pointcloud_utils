#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs import point_cloud2

global points_pub


#input_path = sys.argv[1]
#output_path = sys.argv[2]
#operation = sys.argv[3]

#print("Input path: ", input_path)
#print("Output path: ", output_path)

#input_file = open(input_path, 'r')
def pointcloudCallback(msg):
	global points_pub
	
	#if timestamp from header is not zero, use that!
	timestamp_ros = msg.header.stamp
	timestamp_header = msg.header.stamp.secs * 1000000000 + msg.header.stamp.nsecs
	if (timestamp_header == 0):
		time_now = rospy.get_rostime()
		timestamp_ros = time_now
		timestamp = time_now.secs * 1000000000 + time_now.nsecs
		time_diff = timestamp - timestamp_header
		print "Timestamp difference: " + str(time_diff) + " ns\n"
	else:
		timestamp = timestamp_header

	cloud_2 = msg
	cloud_2.header.stamp = timestamp_ros

	#republish
	points_pub.publish(cloud_2)


def init(topic):
	global points_pub
	points_pub = rospy.Publisher(topic, PointCloud2, queue_size=10)



if __name__ == '__main__':
	rospy.init_node('pointcloud_stamper', anonymous=True)

	pointcloud_topic_in = rospy.get_param('~cloud_topic_in', "/cloud")

	pointcloud_topic_out = rospy.get_param('~cloud_topic_out', "/cloud_stamped")

	init(pointcloud_topic_out)

	queue = 1
		
	rospy.Subscriber(pointcloud_topic_in, PointCloud2, pointcloudCallback, queue_size=queue)

	rospy.spin()