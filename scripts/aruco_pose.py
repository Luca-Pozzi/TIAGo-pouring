#!/usr/bin/env python
# make the robot grasp a certain object

# ROS libs
import tf
import rospy

from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from std_msgs.msg import Header, Float64

import math
import numpy as np
import time

import geometry.points as pts
import geometry.quaternions as quat

CALIBRATION_QUATERNION = [0, 0, 0, 1] #[0, -0.707, 0, 0.707]

class Aruco:
    def __init__(self):
        pass
        '''
        self.aruco_pose = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.displacement)
        self.ideal_pub = rospy.Publisher('/aruco_single/ideal_pose', PoseStamped, queue_size = 1)
        self.calibrated_pose_pub = rospy.Publisher('/aruco_single/displacement_pose', PoseStamped, queue_size = 1)
        self.tilt_pub = rospy.Publisher('/aruco_single/tilt', Float64, queue_size = 1)
        
        self.t_old = time.time()
        self.averaged_samples = 0
        self.time_window = 1 #[s]

        self.mode = 'initial_pose_acquisition' # 'final_pose_acquisition' # 'displacement_evaluation'
        '''

    def displacement(self, pose_msg):
        self.pose_ideal = PoseStamped()
        self.pose_ideal.pose.position = pose_msg.pose.position
        self.ideal_pub.publish(self.pose_ideal)
        if self.averaged_samples == 0:
            self.pose_i = pose_msg
            orientation_i = self.pose_i.pose.orientation
            self.quaternion_calibration = Quaternion(*tf.transformations.quaternion_multiply(tf.transformations.quaternion_conjugate(CALIBRATION_QUATERNION), [orientation_i.x, orientation_i.y, orientation_i.z, orientation_i.w]))
        else:
            pose_f = PoseStamped()
            pose_f.header.frame_id = '/base_footprint'

            point_f = pose_msg.pose.position
            pose_f.pose.position = point_f

            quat_c = self.quaternion_calibration
            quat_f = pose_msg.pose.orientation
            quat_f_c = tf.transformations.quaternion_multiply([quat_f.x, quat_f.y, quat_f.z, quat_f.w], tf.transformations.quaternion_conjugate([quat_c.x, quat_c.y, quat_c.z, quat_c.w]))
            pose_f.pose.orientation = Quaternion(*quat_f_c)
            # publish the results
            #self.calibrated_pose_pub.publish(pose_f)
            
            q1, q2, q3, q0 = quat_f_c
            DCM = np.array([np.array([q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2)]),
                            np.array([2*(q1*q2 - q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 + q0*q1)]),
                            np.array([2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), q0**2 - q1**2 - q2**2 + q3**2])])
            X = DCM[:, 0] 
            Y = DCM[:, 1]
            Z = DCM[:, 2]
            Z = Z / np.linalg.norm(Z)

            tilt = math.atan2(np.linalg.norm(np.cross([0, 0, 1], Z)), np.dot([0, 0, 1], Z))

            if self.averaged_samples == 1:
                self.tilt = tilt
                
            if time.time() - self.t_old > self.time_window:
                self.tilt_pub.publish(self.tilt * 180 / math.pi)
                rospy.loginfo('Inclination: ' + str(self.tilt * 180 / math.pi) + ' degrees.')
                self.t_old = time.time()
                self.averaged_samples = 0
            else:
                self.tilt = (self.tilt * self.averaged_samples +  tilt) / (self.averaged_samples + 1)

        self.averaged_samples += 1

    def aruco_orientation(self, pose_i = None, time_limit = 3):
        ##########
        # Look for ArUco markers, if at least one marker with the correct ID (set at robot startup), return its pose or check the
        # displacement and misalignment if an initial pose is set in input.
        # pose_i:       [Pose] a previous pose of the same marker. If set, the displacement and misalignment of the current
        #               pose are evaluated and returned
        # time_limit:   [float] the maximum time (in [s]) to wait when looking for a marker 
        ##########
        rospy.loginfo('Looking for ArUco markers...')
        averaged_samples = 1
        elapsed_time = 0
        while elapsed_time < time_limit: # set also a minimum (and maximum) number of averaged samples?
            t = rospy.get_time()
            try:
                pose_msg = rospy.wait_for_message('/aruco_single/pose', PoseStamped, timeout = time_limit)
                #rospy.loginfo('Waited for {t:.3f} s to get a marker at {marker_z:.2f}m from floor.'.format( t = rospy.get_time() - t,
                #                                                                                            marker_z = pose_msg.pose.position.z))
                got_aruco = True
            except rospy.exceptions.ROSException: # handle the exception thrown by wait_for_message if timeout occurs
                rospy.logwarn('Timeout exceeded the time limit of {time_limit:.0f}s.'.format(time_limit = time_limit))
                got_aruco = False
            
            if got_aruco and pose_msg.header.frame_id == '/base_footprint':
                if averaged_samples == 1:
                    pose_f = pose_msg.pose
                elif averaged_samples > 1:
                    pose_f.position = pts.average_point(new_point = pose_msg.pose.position, num_samples = averaged_samples, avg_point = pose_f.position)
                    pose_f.orientation = quat.average_Quaternions(new_q = pose_msg.pose.orientation, num_samples = averaged_samples, avg_q = pose_f.orientation)
                averaged_samples += 1
            elapsed_time += rospy.get_time() - t

        if averaged_samples == 1:
            if not pose_i:
                rospy.logwarn('No marker and stop ArUco detection')
            else:
                rospy.logwarn('No marker')
        else:
            # 'unpack' the Quaternion object (tf.transformations methods require a list in input)
            quat_f = [pose_f.orientation.x, pose_f.orientation.y, pose_f.orientation.z, pose_f.orientation.w]
            if not pose_i:
                calibration_pose = PoseStamped()
                calibration_pose.header.frame_id = '/base_footprint'
                calibration_pose.pose.position = pose_f.position
                # return the pose that maps the orientation to the 'ideal' initial orientation (i.e. pointing upward)
                calibration_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_multiply(tf.transformations.quaternion_conjugate(CALIBRATION_QUATERNION), quat_f))
                return calibration_pose
            else:
                calibrated_pose = PoseStamped()
                calibrated_pose.header.frame_id = '/base_footprint'    # set the header of the message
                # evaluate the translation of the marker
                point_i = pose_i.pose.position
                point_f = pose_f.position
                calibrated_pose.pose.position = pose_f.position
                # evaluate the rotation of the marker
                quat_i = [pose_i.pose.orientation.x, pose_i.pose.orientation.y, pose_i.pose.orientation.z, pose_i.pose.orientation.w]
                quat_f_c = tf.transformations.quaternion_multiply(quat_f, tf.transformations.quaternion_conjugate(quat_i))
                calibrated_pose.pose.orientation = Quaternion(*quat_f_c)
                q1, q2, q3, q0 = quat_f_c
                DCM = np.array([np.array([q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2)]),
                                np.array([2*(q1*q2 - q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 + q0*q1)]),
                                np.array([2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), q0**2 - q1**2 - q2**2 + q3**2])])
                X = DCM[:, 0] 
                Y = DCM[:, 1]
                Z = DCM[:, 2]
                Z = Z / np.linalg.norm(Z)

                tilt = math.atan2(np.linalg.norm(np.cross([0, 0, 1], Z)), np.dot([0, 0, 1], Z))
                #rospy.loginfo('Inclination: ' + str(tilt * 180 / math.pi) + ' degrees.')
                        
                return tilt, point_f.z - point_i.z

    def aruco_orientation_from_eul(self, position_i = None, time_limit = 3)
        ##########
        # Look for ArUco markers, if at least one marker with the correct ID (set at robot startup), return its pose or check the
        # displacement and misalignment if an initial pose is set in input.
        # pose_i:       [Pose] a previous pose of the same marker. If set, the displacement and misalignment of the current
        #               pose are evaluated and returned
        # time_limit:   [float] the maximum time (in [s]) to wait when looking for a marker 
        ##########
        rospy.loginfo('Looking for ArUco markers...')
        averaged_samples = 1
        elapsed_time = 0
        while elapsed_time < time_limit: # set also a minimum (and maximum) number of averaged samples?
            t = rospy.get_time()
            try:
                pose_msg = rospy.wait_for_message('/aruco_single/pose', PoseStamped, timeout = time_limit)
                #rospy.loginfo('Waited for {t:.3f} s to get a marker at {marker_z:.2f}m from floor.'.format( t = rospy.get_time() - t,
                #                                                                                            marker_z = pose_msg.pose.position.z))
                got_aruco = True
            except rospy.exceptions.ROSException: # handle the exception thrown by wait_for_message if timeout occurs
                rospy.logwarn('Timeout exceeded the time limit of {time_limit:.0f}s.'.format(time_limit = time_limit))
                got_aruco = False
            
            if got_aruco and pose_msg.header.frame_id == '/base_footprint':
                if averaged_samples == 1:
                    position_f = pose_msg.pose.position
                elif averaged_samples > 1:
                    position_f = pts.average_point(new_point = pose_msg.pose.position, num_samples = averaged_samples, avg_point = position_f)
                averaged_samples += 1
            elapsed_time += rospy.get_time() - t

        if averaged_samples == 1:
            if not pose_i:
                rospy.logwarn('No marker and stop ArUco detection')
            else:
                rospy.logwarn('No marker')
        else:
            if not position_i:
                return position_f
            else:
                # euclidean distance between the two positions
                displacement = math.sqrt((position_i.x - position_f.x) ** 2 + (position_i.y - position_f.y) ** 2 + (position_i.z - position_f.z) ** 2)
                        
                return pose_f, displacement

if __name__ == "__main__":
    rospy.init_node('inclinometer')
    ar = Aruco()
    try:
        #rospy.spin()
        
        calibration_pose = ar.aruco_orientation()
        rospy.logwarn('Tilt now')
        rospy.sleep(3)
        tilt, lift = ar.aruco_orientation(pose_i = calibration_pose)
        rospy.loginfo('Inclination: ' + str(tilt * 180 / math.pi) + ' degrees.\nDisplacement: ' + str(lift))
        
    except KeyboardInterrupt:
        rospy.logwarn('Shutting down the grasping node')