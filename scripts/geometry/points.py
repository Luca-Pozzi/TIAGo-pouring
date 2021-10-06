#!/usr/bin/env python
# some useful operations with Point, PointCloud and PointCloud2 messages

# ROS libs
from sensor_msgs import point_cloud2
# ROS messages
from sensor_msgs.msg import PointCloud, PointCloud2
from segmentation.msg import ObjCloud
from geometry_msgs.msg import PointStamped, Point, Point32
# Python libs
import numpy as np
from operator import attrgetter
from collections import namedtuple


########### BASIC OPERATIONS ##########
def sum_Point(p1, p2):
	return Point(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z)

def divide_Point(p, scalar):
	return Point(p.x / scalar, p.y / scalar, p.z / scalar)

def multiply_Point(p, scalar):
	return Point(p.x * scalar, p.y * scalar, p.z * scalar)
####### end of basic operations #######

########### AVERAGING ##########
def average_point(new_point, num_samples, avg_point): # should be points
	avg_point = divide_Point(sum_Point(multiply_Point(avg_point, num_samples), new_point), (num_samples + 1))
	return avg_point

def average_keyvalues(new_vals, num_samples, avg_vals):
	return [(avg_val * num_samples + new_val) / (num_samples + 1) for avg_val, new_val in zip(avg_vals, new_vals)]
####### end of averaging #######

########### CONVERSIONS ##########
def PointCloud2_to_PointCloud(pcl2_msg):
	### data type handling
	if pcl2_msg._type == 'segmentation/ObjCloud':
		pcl2_msg = pcl2_msg.cloud
	### conversion
	pcl_msg = PointCloud()
	pcl_msg.header = pcl2_msg.header # same header
	# read the points from the PointCloud2
	depth_points_list = point_cloud2.read_points_list(pcl2_msg, skip_nans = True, field_names = ("x", "y", "z"))
	# format the points as Point32 messages and add them to the PointCloud message
	pcl_msg.points = [Point32(point.x, point.y, point.z) for point in depth_points_list]
	return pcl_msg

def PointCloud_to_PointCloud2(pcl_msg):
	return point_cloud2.create_cloud_xyz32(pcl_msg.header, pcl_msg.points)

def datatype_handler(msg):
	if msg._type == 'sensor_msgs/PointCloud':
		cloud = msg
		points_list = msg.points # list of Point32 messages
	else:
		if msg._type == 'segmentation/ObjCloud':
			cloud = msg.cloud
		if msg._type == 'sensor_msgs/PointCloud2':
			cloud = msg
		# convert data in PointCloud2 message to a list of points
		points_list = point_cloud2.read_points_list(cloud, skip_nans = True, field_names = ("x", "y", "z")) # list of namedtuples
	return cloud.header, points_list # maybe it is more meaningful to extract the header instead of "cloud" since the clouds are different
####### end of conversions #######

########## FILTERING ##########
def clean_point_cloud(points_list, direction = 'x', std_dev = 1, threshold = 0.1):
	##########
	# Filter a pointcloud
	##########
	# convert the list of points in a numpy array
	points_array = np.array([[point.x, point.y, point.z] for point in points_list])
	for coord in range(3):
		pass
####### end of filtering ######

########### FEATURES EXTRACTION ##########
def get_keypoints(msg, cluster_size = 10):
	##########
	# NOTE: the centroid is computed averaging the coordinates of the visible points, hence the z coordinate is likely
    #       to be closer to the camera with respect to the real object centroid.
    # NOTE: the centroid is inaccurate in case of obstruction of the line of sight.
    # cluster_size:		[int] number of points averaged to get a keyvalue.
    #					EXAMPLE: with a cluster size of 10, the xmin keyvalue is the average of the x values of the 10
    #					points with lower x value. The same applies to all the joints
	##########
	header, depth_points_list = datatype_handler(msg)
	# convert the list of Points into a numpy ndarray
	depth_points_array = np.array([[point.x, point.y, point.z] for point in depth_points_list])
	# extract the object centroid (average of each coordinate)
	centroid = np.array([depth_points_array[:, 0].mean(), depth_points_array[:, 1].mean(), depth_points_array[:,2].mean()])
	centroid = PointStamped(header, Point(centroid[0], centroid[1], centroid[2]))
	# get keyvalues
	keyvalues = []
	for coord in range(3):
		sorted_array = np.sort(depth_points_array[:, coord])
		keyvalues.extend([sorted_array[:cluster_size].mean(), sorted_array[-cluster_size:].mean()])
	'''
	keyvalues = [depth_points_array[:, 0].min(), depth_points_array[:, 0].max(),	# xmin, xmax
				depth_points_array[:, 1].min(), depth_points_array[:, 1].max(),		# ymin, ymax
				depth_points_array[:, 2].min(), depth_points_array[:, 2].max()]		# zmin, zmax
	'''
	return depth_points_array, centroid, keyvalues

def keyvalues_as_namedtuple(keyvals):
	Keyvalues = namedtuple('Keyvalues', ['xmin', 'xmax', 'ymin', 'ymax', 'zmin', 'zmax'])
	return Keyvalues(keyvals[0], keyvals[1], keyvals[2], keyvals[3], keyvals[4], keyvals[5])