#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs import point_cloud2

global points_pub
global average_delay


#input_path = sys.argv[1]
#output_path = sys.argv[2]
#operation = sys.argv[3]

#print("Input path: ", input_path)
#print("Output path: ", output_path)

#input_file = open(input_path, 'r')
def pointcloudCallback(msg):
	global points_pub

	#print("delay: ")
	#print(average_delay)
	#print("\n")
	
	#if timestamp from header is not zero, use that!
	timestamp_ros = msg.header.stamp
	timestamp_full = msg.header.stamp.secs * 1000000000 + msg.header.stamp.nsecs
	if (timestamp_full == 0):
		time_now = rospy.get_rostime()
		timestamp_ros = time_now

		timestamp = (time_now.secs - average_delay) * 1000000000 + time_now.nsecs
		timestamp_secs = int(timestamp) / 1000000000
		timestamp_nsecs = timestamp - timestamp_secs * 1000000000

		#print(str(timestamp_nsecs))
		#timestamp = timestamp_secs * 1000000000 + timestamp_nsecs
		time_diff = timestamp - timestamp_full
		#print "Timestamp difference: " + str(time_diff) + " ns\n"
	else:
		#print("Adjusting current stamp\n")
		timestamp = (timestamp_ros.secs - average_delay) * 1000000000 + timestamp_ros.nsecs
		#timestamp_secs = timestamp_ros.secs - average_delay
		#timestamp_nsecs = timestamp_ros.nsecs
		timestamp_secs = int(timestamp)  / 1000000000
		timestamp_nsecs = timestamp - timestamp_secs * 1000000000

	cloud_2 = msg
	timestamp_ros.secs = timestamp_secs #timestamp / 1000000000
	timestamp_ros.nsecs = timestamp_nsecs #timestamp - (timestamp_ros.secs * 1000000000)
	#print(timestamp_header)
	cloud_2.header.stamp = timestamp_ros

	#republish
	points_pub.publish(cloud_2)


def init(topic):
	global points_pub
	global average_delay

	average_delay = rospy.get_param('~average_delay', 0.033)
	points_pub = rospy.Publisher(topic, PointCloud2, queue_size=10)
1000000000


if __name__ == '__main__':
	rospy.init_node('pointcloud_stamper', anonymous=True)

	pointcloud_topic_in = rospy.get_param('~cloud_topic_in', "/cloud")

	pointcloud_topic_out = rospy.get_param('~cloud_topic_out', "/cloud_stamped")

	init(pointcloud_topic_out)

	queue = 1
		
	rospy.Subscriber(pointcloud_topic_in, PointCloud2, pointcloudCallback, queue_size=queue)

	rospy.spin()