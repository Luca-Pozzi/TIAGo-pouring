#!/usr/bin/env python
# define some function to deal with quaternions.
# TODO: 
#		make it work both with Quaternion messages and a quat in an order specified by the user
#		the apply_rotation_to_point should become apply_rotation and accept Point, Vector

from geometry_msgs.msg import Quaternion, Point, PointStamped
import math

def conjugate(q):
	# return the conjugate of the given quaternion ()
	return Quaternion(-q.x, -q.y, -q.z, q.w)

def multiply(q1, q2):
	# multiply the two given quaternions
	w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
	x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
	y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z
	z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x
	return Quaternion(x, y, z, w)

def list_to_Quaternion(q, order = 'xyzw'): # implement a routine to let the user specify the order of the values
    return Quaternion(q[0], q[1], q[2], q[3])

def apply_rotation_to_point(q, p):
	# apply the rotation specified by the quaternion to a point (Point)
	pre_multiply = multiply(q, Quaternion(p.x, p.y, p.z, 0))
	post_multiply = multiply(pre_multiply, conjugate(q))
	return Point(post_multiply.x, post_multiply.y, post_multiply.z)

def angle_between_Quaternions(q1, q2):
    try:
        return 2 * math.acos(q1.w + q2.w + q1.x + q2.x + q1.y + q2.y + q1.z + q2.z)
    except ValueError:
        return math.pi / 2

def average_Quaternions(new_q, num_samples, avg_q):
    return Quaternion(  (avg_q.x * num_samples + new_q.x) / (num_samples + 1),
                        (avg_q.y * num_samples + new_q.y) / (num_samples + 1),
                        (avg_q.z * num_samples + new_q.z) / (num_samples + 1),
                        (avg_q.w * num_samples + new_q.w) / (num_samples + 1))

def transform_point(frame, point_stamped, translation, rotation):
    point_transformed = PointStamped()
    # the header of the new point must be the same as before...
    point_transformed.header.seq = point_stamped.header.seq
    point_transformed.header.stamp = point_stamped.header.stamp
    # ...except for the frame
    point_transformed.header.frame_id = frame
    # the point is rotated as specified by the quaternion
    rot_point = apply_rotation_to_point(Quaternion(rotation[0], rotation[1], rotation[2], rotation[3]), point_stamped.point)
    # translate the Point
    point_transformed.point.x = rot_point.x + translation[0]
    point_transformed.point.y = rot_point.y + translation[1]
    point_transformed.point.z = rot_point.z + translation[2]
    return point_transformed

def euler_from_Quaternion(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])

def euler_from_quaternion(x, y, z, w):
    # from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z # in radians