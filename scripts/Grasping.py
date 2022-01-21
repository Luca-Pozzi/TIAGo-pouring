#!/usr/bin/env python
# make the robot grasp a certain object

# ROS libs
import tf
import rospy, rospkg
import moveit_commander 
# ROS messages
from segmentation.msg import ObjCloud
from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion, PointStamped, Pose, Point, PoseStamped, Vector3, Vector3Stamped
from sensor_msgs.msg import CompressedImage
from moveit_msgs.msg import RobotTrajectory 
from trajectory_msgs.msg import JointTrajectoryPoint
# Python libs
import sys, os
import cv2
import datetime
import yaml, csv
import numpy as np
import math
# my scripts
import geometry.points as pts
import geometry.quaternions as quat
from ros_modules import roslauncher
from ros_modules.Playmotion import Playmotion
import ros_modules.moveit_modules.movements as move
from ros_modules.moveit_modules.planning_scene import CollisionManager, ConstraintManager, load_moveit_config
from utils.errors import PointcloudException, ArucoException, PlanningException

BOTTLE_MIN_RADIUS = 0.0225 # not used
BOTTLE_MIN_RADIUS_HEIGHT = 0.115

class Grasping:
    def __init__(self):
        ### Locate the current package in the filesystem
        rospack = rospkg.RosPack()  # get an instance of RosPack with the defaul search paths
        self.path_to_package = rospack.get_path('tiago_pouring')
        
        ### Playmotion and MoveIt utilities
        self.playmotion = Playmotion()
        self.collision_manager = CollisionManager()
        self.constraint_manager = ConstraintManager()
        
        ### TF
        # create a Transform Listener that subscribes to the /tf topic and allows to transform ROS messages
        # from one reference frame to another 
        self.tf_listener = tf.TransformListener(cache_time = rospy.Duration(10))
        rospy.loginfo('SimpleActionClient and TransformListener initialized.')
        
        ### MoveIt
        # initialize a moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        # exploit MoveIt to move TIAGo
        self.robot = moveit_commander.RobotCommander()  # the interface with the robot
        self.scene = moveit_commander.PlanningSceneInterface()  # the interface with the world surrounding the robot
        # define a commander for each group of the robot (if all of them are needed)
        self.hand = moveit_commander.MoveGroupCommander('hand')
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.arm_torso = moveit_commander.MoveGroupCommander('arm_torso')
        rospy.loginfo('MoveIt commander initialized.\n')
        
        ### Load configuration
        config_file = rospy.get_param('/tiago_grasp/config')
        with open(config_file) as file:
            config = yaml.load(file)
        self.simulation = config['simulation']          # is the code running in simulation or on the real robot?
        self.learning = config.get('learning', False)   # is TIAGo performing a single task or an episode of an RL process?
        # build/clear scene parameters
        self.TIME_LIMIT = config.get('look_for', 5)             # maximum time to wait when looking for the object or for ArUco markers
        self.build_full_scene = config.get('full_setup', True)  # if False, the 'explore_surroundings' movement is not performed
        self.clear_all = config.get('clear_octomap', False) 

        # what to check/export
        self.save = config.get('save_results', True) 
        self.aruco = config.get('aruco', False) if not self.simulation else False # if the code is run in simulation, force the flag to False
        self.pictures = config.get('pictures', False)
        
        # motion strategies
        self.open_wide_right = config.get('open_wide_right', 'true')    # if True, before approaching the object, TIAGo stretches its arm to the right
        if self.open_wide_right:
             self.arm.remember_joint_values('open_wide_right', values = [math.pi/40, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.arm.remember_joint_values('pre_tilt_pose', values = [math.pi/4, 0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/4, 0.0])
        #self.arm.remember_joint_values('pre_tilt_pose', values = [math.pi/4, -math.pi/18, -math.pi/2, math.pi*5/9, -math.pi*5/9, -math.pi*5/18, -math.pi/36])
        #self.arm.remember_joint_values('pre_tilt_pose', values = [math.pi/3, 0.0, -math.pi/2, math.pi/3, -math.pi/2, -math.pi/6, 0.0])
        #self.arm.remember_joint_values('pre_tilt_pose', values = [math.pi/3, -math.pi/18, -math.pi/2, math.pi/3, -math.pi*7/12, -math.pi/6, math.pi/36])
        self.pregrasp = config.get('pregrasp_strategy', 'open')
        self.approach = config.get('approach_strategy', 'close')
        self.eef_orientation = config.get('eef_orientation', [0, 0, 0, 1])
        self.grasp_height = config.get('grasp_height', 'half')
        self.grasp_joint_values = config.get('grasp_joint_values', None)
        self.grasp_height_delta = config.get('grasp_height_delta', None)
        self.current_escape = None  # the escape_strategy remains None (i.e. 'stay still') until the eef is placed close to the object
        self.desired_escape = config.get('escape_strategy', 'nearby')
        if self.desired_escape == 'home':
            self.arm.remember_joint_values('tuck', values = [0.191986, -1.3439, -0.191986, 1.93732, -math.pi/2, 1.37881, 0.0])

        # MoveIt planning parameters
        self.set_visibility_constraint = False # the visibility constraint is too strict, consider to just get rid of it
        self.moveit_config = config.get('moveit_config', {}) 

        # keep track of the results
        current_datetime = datetime.datetime.now() #tz = datetime.timezone(offset = datetime.timedelta(hours = 1)))
        self.today_string = str(current_datetime.year) + '_' + str(current_datetime.month) + '_' + str(current_datetime.day)
        self.now_string = str(current_datetime.hour) + '_' + str(current_datetime.minute) + '_' + str(current_datetime.second)

        self.episode_results = dict(timestamp = self.today_string + self.now_string,                               # timestamp
                                        centroid = None, keyvals = None, height = None, radius = None, center = None,   # geometric features
                                        desired_grasp_point = None, actual_grasp_point = None,                          # grasp point
                                        joint_values = None, joint_absolute_errors = None, joint_relative_errors = None,# joint values
                                        lift_height = None, lift = None, misalignment_lift = None,                      # lift outcome
                                        small_tilt = None, large_tilt = None,                                           # manipulation outcome
                                        misalignment_place = None,                                                      # place outcome
                                        fall = False, when_fall_happened = None                                             # info about failure    
                                        )

    def get_target_coord(self, launcher, obj_class, to_frame, time_limit = 5):
        ##########
        # Launch the nodes for semantic segmentation (exploiting a mask-RCNN). If one of the predicted classes is obj_class,
        # the geometrical features of that object are retrieved.
        # NOTE: this method should be called when the robot is looking at the desired object, there is no search for it
        # launcher:     [roslaunch object] generated from roslaunch API function. Used for shutting down the object
        #               segmentation nodes when they are no more required.
        # obj_class:    [string] the class of the object to look for (as returned by the segmentation algorithm) that will
        #               also be used to name the cylinder added to the planning scene
        # to_frame:     [string] the frame in which the position of the object must be brought
        # time_limit:   [float] duration (in [s]) of the time interval in which the robot collects pointcloud samples of the
        #               selected object 
        ##########
        # initialize the number of samples and the timer
        averaged_samples = 1
        tf_sync_errors = 0
        start_time = rospy.get_time()
        elapsed_time = 0
        rospy.loginfo('Looking for ' + obj_class + '...')
        print obj_class.capitalize() + ' pointclouds received: ',
        sys.stdout.flush()
        # collect some output of the segmentation pipeline and average the results
        while elapsed_time < time_limit: # set also a minimum (and maximum) number of averaged samples?
            t = rospy.get_time() # evaluate processing time
            try:
                # subscribe to the "/segmentation_result/cloud/semantic", read a message and then unsubscribe
                target = rospy.wait_for_message("/segmentation_result/cloud/semantic", ObjCloud, timeout = time_limit)
                print '*',
                sys.stdout.flush()
                got_cloud = True
            except rospy.exceptions.ROSException:
                print('\n')
                rospy.logwarn('Timeout exceeded the time limit of {time_limit:.0f}s.'.format(time_limit = time_limit))
                got_cloud = False

            if got_cloud and target.obj_class == str(obj_class): # check that the object belongs to the selected class
                # convert the target cloud to a PointCloud message to make it suitable for transformPointCloud
                target = pts.PointCloud2_to_PointCloud(target)
                try:
                    self.tf_listener.waitForTransform('xtion_rgb_optical_frame', 'base_footprint', rospy.Time(), rospy.Duration(5))    
                    target = self.tf_listener.transformPointCloud(to_frame, target) 
                    transformed_cloud = True # the flag could be modified into got_tf and the cloud transformation brought into the following if block
                except tf.ExtrapolationException:
                    #rospy.logwarn('tf raised an error due to synchronization.')
                    tf_sync_errors += 1
                    transformed_cloud = False

                if transformed_cloud:   # if the transformation went smooth, get the object keypoints
                    __, centroid, keyvals = pts.get_keypoints(target)   # centroid: [PointStamped], keyvals: [namedtuple]
                                                                        # __ is used to ignore unused return
                    if averaged_samples == 1:   # if it is the very first sample acquired, initialize the average to these values
                        avg_centroid = centroid
                        avg_keyvals = keyvals
                    elif averaged_samples > 1:  # if it is not the first sample, average the results to add robustness
                        avg_centroid.point = pts.average_point(centroid.point, averaged_samples, avg_centroid.point)
                        avg_keyvals = pts.average_keyvalues(keyvals, averaged_samples, avg_keyvals)
                    averaged_samples += 1

            elapsed_time = rospy.get_time() - start_time    # evaluate from how long the while loop is running
        print('\n')

        if launcher:
            launcher.shutdown()   # shut down the nodes for vision (if is not None)
            print('\n')
        if averaged_samples > 1: # if at least 1 valid pointcloud has been received
            if tf_sync_errors:
                rospy.logwarn('{errors} out of {total} messages received were rejected because of TF synchronization issues.'.format(errors = tf_sync_errors,
                                                                                                                                    total = tf_sync_errors + averaged_samples - 1))
            rospy.loginfo('Elapsed_time: {elapsed_time:.2f} \nAveraged samples: {samples} \nCentroid: [{x:.2f}, {y:.2f}, {z:.2f}]'.format(elapsed_time = elapsed_time, 
                                                                                                                                            samples = averaged_samples - 1, 
                                                                                                                                            x = avg_centroid.point.x, 
                                                                                                                                            y = avg_centroid.point.y, 
                                                                                                                                            z = avg_centroid.point.z))
            # save keyvals as namedtuple with fields ['xmin', 'xmax', 'ymin', 'ymax', 'zmin', 'zmax'] to hopefully make the code more readable
            avg_keyvals = pts.keyvalues_as_namedtuple(avg_keyvals)
            # compute the center, height and radius of the object (approximated as a cylinder)
            height = avg_keyvals.zmax - avg_keyvals.zmin if avg_keyvals.zmax - avg_keyvals.zmin > 0.20 else 0.20
            radius = (avg_keyvals.ymax - avg_keyvals.ymin) / 2 if (avg_keyvals.ymax - avg_keyvals.ymin) / 2 < 0.03 else 0.03
            center = Point(avg_keyvals.xmin + radius, avg_centroid.point.y, self.table_height + height / 2) # avg_keyvals.ymin + radius for Y
            # add a cylinder with the name, dimension and pose of the object to the planning scene
            self.scene.add_cylinder(obj_class,                                                              # name
                                    PoseStamped(centroid.header, Pose(center, Quaternion(0, 0, 0, 1))),     # pose
                                    height = height, radius = radius)                                       # size
            rospy.loginfo('Cylinder of height {h:.2f} and radius {r:.2f} added to planning scene at point [{x:.2f}, {y:.2f}, {z:.2f}]\n'.format(h = height,
                                                                                                                                                r = radius, 
                                                                                                                                                x = center.x,
                                                                                                                                                y = center.y,
                                                                                                                                                z = center.z))
            if self.aruco:
                self.scene.add_cylinder('aruco',
                                        PoseStamped(centroid.header, Pose(Point(center.x, center.y, center.z + height / 2 + 0.15 / 2),
                                                                        Quaternion(0, 0, 0, 1))),
                                        height = 0.15, radius = 0.035) # hard-coded measures of the marker
            # save the results
            self.episode_results['centroid'] = [round(avg_centroid.point.x, 4), round(avg_centroid.point.y, 4), round(avg_centroid.point.z, 4)]
            self.episode_results['keyvals'] = [round(keyval, 4) for keyval in avg_keyvals]
            self.episode_results['height'] = round(height, 4)
            self.episode_results['radius'] = round(radius, 4)
            self.episode_results['center'] = [round(center.x, 4), round(center.y, 4), round(center.z, 4)]
            return avg_centroid, avg_keyvals # PointStamped, namedtuple([float])
        elif averaged_samples == 1: # i.e. if no valid pointcloud has been received
            raise PointcloudException
    
    def aruco_orientation(self, pose_i = None, time_limit = 3, max_samples = 10):
        ##########
        # Look for ArUco markers, if at least one marker with the correct ID (set at robot startup), return its pose or check the
        # displacement and misalignment if an initial pose is set in input.
        # pose_i:       [PoseStamped] a previous pose of the same marker. If set, the displacement and misalignment of the current
        #               pose are evaluated and returned
        # time_limit:   [float] the maximum time (in [s]) to wait when looking for a marker 
        ##########
        CALIBRATION_QUATERNION = [0, 0, 0, 1]
        rospy.loginfo('Looking for ArUco markers...')
        print('Markers found: '),
        sys.stdout.flush()
        averaged_samples = 1
        elapsed_time = 0
        while elapsed_time < time_limit and averaged_samples <= max_samples: # set also a minimum (and maximum) number of averaged samples?
            t = rospy.get_time()
            try:
                pose_msg = rospy.wait_for_message('/aruco_single/pose', PoseStamped, timeout = time_limit)
                print('*'),
                sys.stdout.flush()
                got_aruco = True
            except rospy.exceptions.ROSException: # handle the exception thrown by wait_for_message if timeout occurs
                print('\n') # since it comes after a print ending without newline
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
        print('\n')

        if averaged_samples == 1: # if no marker has been found before the timer elapsed
            if pose_i:
                return None, None
            else:
                return None
        else:
            rospy.loginfo('\nElapsed_time: {elapsed_time:.2f} \nAveraged samples: {samples} \nPosition: [{x:.2f}, {y:.2f}, {z:.2f}]'.format(elapsed_time = elapsed_time, 
                                                                                                                                            samples = averaged_samples - 1, 
                                                                                                                                            x = pose_f.position.x, 
                                                                                                                                            y = pose_f.position.y, 
                                                                                                                                            z = pose_f.position.z))
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

                tilt = abs(math.atan2(np.linalg.norm(np.cross([0, 0, 1], Z)), np.dot([0, 0, 1], Z)))
                rospy.loginfo('Inclination: {angle:.0f} degrees.\n'.format(angle = tilt * 180 / math.pi))
                        
                return tilt, point_f.z

    def build_scene(self, obj_class):
        ##########
        # Move the robot's head to have a clear view of the scene. Exploit segmentation nodes to extract meaningful objects
        # and add them to the MoveIt! planning scene.
        # obj_class:    [string] the class of the object to look for (as returned by the segmentation algorithm)
        ##########
         # launch the nodes for vision (if needed)
        segmentation_launch_file = rospy.get_param('/tiago_grasp/segmentation_launch_file', None) if rospy.get_param('/tiago_grasp/as_node') else None # if the code is not run 'as_node', force to False
        score_detection_threshold = 'detection_score_threshold:=0.25' if self.simulation else 'detection_score_threshold:=0.6'
        object_segmentation_launcher = roslauncher.roslaunch_from_file(segmentation_launch_file, [score_detection_threshold]) if segmentation_launch_file else None

        # In case there is something attached from the previous execution
        self.scene.remove_attached_object(link = 'hand_grasping_frame')
        self.scene.remove_world_object('bottle')
        self.scene.remove_world_object('aruco')

       

        print('Waiting')
        '''
        for i in range(100000):
            pass
        '''
        rospy.sleep(10)
        print('Ok, we can work now')

        move.move_joints(self.arm_torso, joint_values = [0.30], joints_to_move = ['torso_lift_joint'])  # extend the torso to look the scene from above
        self.playmotion.playmotion_movement('look_down')                                                # point the head toward the table to get the target coordinates
        rospy.sleep(2)  #[s] wait for the measure from pointcloud to stabilize            
        self.table_height = 0.8 if self.simulation else 0.77 # get the table height with RANSAC plane segmentation
        # get points of the selected objects that are useful for planning and manipulation
        centroid, keyvalues = self.get_target_coord(object_segmentation_launcher, obj_class, self.arm.get_planning_frame(), time_limit = self.TIME_LIMIT)
        self.bottle_top = keyvalues.zmax # useful to check if the bottle fell while approaching
        if self.build_full_scene:
            self.playmotion.playmotion_movement('explore_surroundings', timeout = 30.0)                 # make the robot move its head all around to build the octomap
        
        # apply pre-defined constraints (see moveit_modules/planning_scene for further informations)
        self.constraints = self.constraint_manager.set_constraints(centroid, keyvalues, visibility = self.set_visibility_constraint) # maybe improve the constraints... the elbow should not "point upward"
        
        if self.aruco:  # if the object is endowed with markers, save their orientation to check displacements later on
            self.object_pose_initial = self.aruco_orientation(time_limit = self.TIME_LIMIT)
            if not self.object_pose_initial:
                rospy.logwarn('No ArUco marker found, it would be impossible to check for misalignment.')
        # set the height of the grasping point (currently only 'lower_third' and the default 'half' are available)
        grasp_point_height = keyvalues.zmin + (keyvalues.zmax - keyvalues.zmin) / 3 if self.grasp_height == 'lower_third' else (keyvalues.zmin + keyvalues.zmax) / 2
        # return the grasp point
        return Point(keyvalues.xmin + 0.015, #centroid.point.x,      # at half the object 'depth'                 
                    centroid.point.y - 0.02, #keyvalues.ymin - 0.001, # to the right of the object
                    grasp_point_height)     # at the selected height
        
    def approach_object(self, grasp_point, grasp_orientation):
        ##########
        # Move the end-effector to the given pose.
        # grasp_point:          [PointStamped] the point to which the origin of the eef frame should be placed
        # grasp_orientation:    [Quaternion] the orientation of the eef when it reaches the grasp_point
        ##########
        # disable collisions with the hand safety box link
        self.collision_manager.allow_collisions(disable_collision_links = ['hand_safety_box'])
        self.arm.set_end_effector_link('hand_grasping_frame') # the default eef link is the one of the wrist
        load_moveit_config(self.arm, self.moveit_config) # set the planning parameters
        # do some preliminary operation
        if self.open_wide_right: 
            move.execute_remembered_joint_values(self.arm, 'open_wide_right')     # stretch the arm to the right
        self.playmotion.playmotion_movement(self.pregrasp)                  # put the hand in a suitable configuration
        self.arm.set_path_constraints(self.constraints)
        
        # add a table object to increase safety against collisions
        if self.build_full_scene: #if not self.learning:
            box_pose = PoseStamped()
            box_pose.header.frame_id = 'base_footprint'
            box_pose.pose.position = Point(1.0, 0.0, self.table_height/2)
            box_pose.pose.orientation = Quaternion(0, 0, 0, 1)
            self.scene.add_box('table', pose = box_pose, size = (1.2, 2.5, self.table_height))
            rospy.loginfo('Box added')

        # adapt the grasp point if needed and plan to move the eef there
        if self.approach == 'straight_from_right':
            grasp_point.y = grasp_point.y - 0.05
        
        # bring the hand close to the desired point with the desired position
        eef_positioning_success, actual_grasp_point = move.eef_to_valid_pose(self.arm, grasp_point, grasp_orientation, exploration_direction = 'z', shift_amount = 0.01, max_shift = 0.10)
        if eef_positioning_success:
            self.current_escape = self.desired_escape
            if not self.simulation: # why?    
                # move the hand forward
                move.straight_eef_movement(self.arm, direction = 'x', translation = 0.02, avoid_collisions = False)
            if self.approach == 'straight_from_right':
                move.straight_eef_movement(self.arm, direction = 'y', translation = 0.05, avoid_collisions = False, velocity_downscaling = 3.0)
            #if self.save:
            self.episode_results['desired_grasp_point'] = [round(grasp_point.x, 4), round(grasp_point.y, 4), round(grasp_point.z, 4)]
            self.episode_results['actual_grasp_point'] = [round(actual_grasp_point.x, 4), round(actual_grasp_point.y, 4), round(actual_grasp_point.z, 4)]
        else: # if no valid plan is found
            raise PlanningException

        if self.aruco:
            __, height_after_approach = self.aruco_orientation(self.object_pose_initial, time_limit = self.TIME_LIMIT)
            if not height_after_approach:
                rospy.logerr('No ArUco marker found, it would be impossible to check for lift success.')
                raise ArucoException
            elif height_after_approach < self.bottle_top:
                self.episode_results['fall'] , self.episode_results['when_fall_happened'] = True, 'approach'
                rospy.logerr('Bottle fallen during hand approach. The task cannot go on.')
                if self.learning:
                    rospy.logerr('The task is considered unmeaningful, since the grasp was not even attempted.')
                raise ArucoException
            else:
                rospy.loginfo('Successfully approached the object.')
        return actual_grasp_point

    def grasp_object(self, object_name, joint_values = []):
        ##########
        # Close the hand following the selected strategy.
        # object_name:  [string] the name of the object to grasp
        # joint_values: [[float]] joint values of the hand joints
        #               NOTE: the joint_values are executed only if self.learning is True, otherwise a default grasp
        #               is attempted.
        ##########
        # allow collision between the object and the links of the grasper (with a terrible workaround)
        #self.hand.attach_object(object_name, link_name = 'hand_grasping_frame', touch_links = self.robot.get_link_names(group = 'hand')) #touch_links = self.hand.get_joints())
        self.scene.remove_world_object(name = 'bottle')
        if self.aruco:
            self.hand.attach_object('aruco', link_name = 'hand_grasping_frame', touch_links = self.robot.get_link_names(group = 'hand')) #touch_links = self.hand.get_joints())
        # check if the joint values for the grasp are set, either as a Grasping attribute through config or as a grasp_object argument
        joint_values = self.grasp_joint_values if self.grasp_joint_values and not joint_values else joint_values
        if not joint_values:
            # close the hand follwing a pre-defined sequence of movements
            self.playmotion.playmotion_movement('grasp', planning = False) # the only line that should be here
            rospy.sleep(2)
            if self.learning:
                rospy.logwarn('Learning is set to True, but joint_values are not specified.')
        else: # elif self.learning and joint_values:
            # close the hand with the joint values defined either by the learning algorithm or by the user
            rospy.loginfo('Attempting grasp with joints: ' + str(joint_values) + ' for []')
            __, joint_abs_err, joint_rel_err = move.move_joints(self.hand, joint_values = joint_values, joints_to_move = self.hand.get_active_joints())
            # store the results
            self.episode_results['joint_values'] = [round(val, 4) for val in joint_values]
            self.episode_results['joint_absolute_errors'] = [round(abs_err, 4) for abs_err in joint_abs_err]
            self.episode_results['joint_relative_errors'] = [round(rel_err, 4) for rel_err in joint_rel_err]
            rospy.sleep(2)

    def take_a_pic(self, name = 'pic'): # could be useful to make it a service
        ##########
        # Take a picture of the scene seen by the robot RGB camera and save it as .jpg file in the 'saved' folder of the package.
        # name:     [string] the name to attach to the timestamp.
        #           NOTE: leaving the default name is likely to cause overwritings.
        ##########
        rospy.loginfo('Going to take a picture')
        img_msg = rospy.wait_for_message('/xtion/rgb/image_rect_color/compressed', CompressedImage)
        # Conversion to cv2 image
        np_arr = np.fromstring(img_msg.data, np.uint8)
        img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        path_to_file = os.path.join(self.path_to_package, 'saved', self.today_string + '_' + self.now_string + '_' + name + '.jpg')
        cv2.imwrite(path_to_file, img_np)
        rospy.loginfo('Image saved at ' + path_to_file)

    def lift(self):
        ##########
        # Raise the end-effector (supposed to be closed with an object in it), check if a marker is attached to it so to
        # measure the displacement. Bring the end-effector down and then far from the object. Again, if a marker is present
        # check the object pose.
        ##########
        # check the marker position to avoid the need of taking tilt in consideration
        lift_height = None
        if self.aruco:
            object_pose_after_grasp = self.aruco_orientation(time_limit = self.TIME_LIMIT)
            if not object_pose_after_grasp:
                rospy.logerr('Bottle fallen while grasping. Task failed.')
                self.episode_results['fall'], self.episode_results['when_fall_happened'] = True, 'grasp'
            elif object_pose_after_grasp.pose.position.z < self.bottle_top:
                rospy.logerr('Bottle fallen while grasping. Task failed.')
                self.episode_results['fall'], self.episode_results['when_fall_happened'] = True, 'grasp' # if the bottle falls during grasp it IS meaningful
            else:
                rospy.loginfo('Marker height after grasp: ' + str(object_pose_after_grasp.pose.position.z))
        
        if self.episode_results['fall']:
            self.episode_results['lift'] = False
        else:
            # lift the object
            move.straight_eef_movement(self.arm, direction = 'z', translation = 0.05, avoid_collisions = False)
            rospy.sleep(2)
            if self.pictures:
                # take a picture of the scene
                self.take_a_pic(name = 'lift')
            # check where the position and orientation of the marker
            if self.aruco:
                self.episode_results['misalignment_lift'], height_after_lift_attempt = self.aruco_orientation(self.object_pose_initial, time_limit = self.TIME_LIMIT)
                if height_after_lift_attempt:
                    rospy.loginfo('Marker height after lift: ' + str(height_after_lift_attempt))
                    self.object_height_after_grasp = object_pose_after_grasp.pose.position.z
                    lift_height = height_after_lift_attempt - self.object_height_after_grasp
                    self.episode_results['lift_height'] = lift_height # save the upward displacement of the bottle
                    if lift_height >= 0.015: # make it adaptive (i.e. a certain percentage of the upward movement) and maybe take into account std to evaluate how 'sure' the measure is
                        rospy.loginfo('Successful lift.')
                        self.episode_results['lift'] = True
                    elif height_after_lift_attempt < self.bottle_top:
                        print('Bottle is at ' + str(height_after_lift_attempt))
                        rospy.loginfo('Lift failed. Bottle fallen.')
                        self.episode_results['lift'], self.episode_results['fall'], self.episode_results['when_fall_happened'] = False, True, 'lift'
                    else:
                        rospy.loginfo('Lift failed.')
                        self.episode_results['lift'] = False
                else:
                    rospy.logwarn('No ArUco marker found, the lift is considered as failed.')
                    self.episode_results['lift'] = False
                    '''
                    rospy.logerr('No ArUco marker found, it would be impossible to check for lift success.') # pr just consider it failed
                    raise ArucoException
                    '''
        #if self.save:
        # if self.episode_results['lift'] else None 
    
    def manipulation(self):

        move.move_joints(self.arm_torso, joint_values = [0.35], joints_to_move = ['torso_lift_joint']) #self.extend_torso(elevation = 0.35)

        joint_values_after_lift = self.arm.get_current_joint_values()
        # get rid of constraints (and maybe set new ones?)
        self.arm.clear_path_constraints() # the constraint "keep the eef pointing upward" should be added
        # move to a known joint configuration which is both safe and allows to tilt the wrist
        move.execute_remembered_joint_values(self.arm, 'pre_tilt_pose')

        

        initial_joint_value = self.arm.get_current_joint_values()[-1]
        small_tilt = 0.7854 # = 45 degrees
        final_joint_value = initial_joint_value - small_tilt
        # tilt the wrist 45 degrees
        success, __, __ = move.move_joints(self.arm, joint_values = [final_joint_value], joints_to_move = ['arm_7_joint'])

        if success:
            if self.aruco: # in principle ok, but check that the marker are visible during manipulation
                # where is the marker?
                __, height_after_tilt_attempt = self.aruco_orientation(self.object_pose_initial, time_limit = self.TIME_LIMIT)
                if height_after_tilt_attempt:
                    if height_after_tilt_attempt < self.bottle_top:
                        self.episode_results['small_tilt'], self.episode_results['fall'], self.episode_results['when_fall_happened'] = False, True, 'small_tilt'
                        rospy.logwarn('The object fell during manipulation phase')
                    else:
                        lift_height_after_manipulation = height_after_tilt_attempt - self.object_height_after_grasp
                        if lift_height_after_manipulation >= 0.015: # make it adaptive (i.e. a certain percentage of the upward movement) and maybe take into account std to evaluate how 'sure' the measure is
                            rospy.loginfo('Successful manipulation.')
                            self.episode_results['small_tilt'] = True
                        else:
                            rospy.loginfo('Unsuccessful manipulation.') # how may this happen?
                            self.episode_results['small_tilt'], self.episode_results['fall'], self.episode_results['when_fall_happened'] = False, True, 'small_tilt'
                else:
                    rospy.logwarn('The marker is not visible, the object is considered as fallen.')
                    self.episode_results['small_tilt'], self.episode_results['fall'], self.episode_results['when_fall_happened'] = False, True, 'small_tilt' # if the marker is not visible, consider the bottle as fallen
        else:
            rospy.logwarn('Tilt was not attempted and the reward/penalty for tilt are not going to be assigned.')

        if self.episode_results['small_tilt'] or not self.aruco: # if the bottle is still in the robot hand, try to tilt it further
            large_tilt = 1.745 # = 100 degrees to see if the liquid in the bottle could be poured while holding it
            final_joint_value = initial_joint_value - large_tilt
            # tilt the wrist as if TIAGo was pouring the liquid in the bottle
            success, __, __ = move.move_joints(self.arm, joint_values = [final_joint_value], joints_to_move = ['arm_7_joint'])
            # go back to the previous joint configuration
            move.move_joints(self.arm, joint_values = joint_values_after_lift, joints_to_move = self.arm.get_active_joints())
            self.arm.set_path_constraints(self.constraints) # restore the constraints
            
            move.move_joints(self.arm_torso, joint_values = [0.30], joints_to_move = ['torso_lift_joint']) #self.extend_torso(elevation = 0.35)

            if success:
                if self.aruco:
                    # where is the marker?
                    __, height_after_tilt_attempt = self.aruco_orientation(self.object_pose_initial, time_limit = self.TIME_LIMIT)
                    if height_after_tilt_attempt:
                        if height_after_tilt_attempt < self.bottle_top:
                            self.episode_results['large_tilt'], self.episode_results['fall'], self.episode_results['when_fall_happened'] = False, True, 'large_tilt'
                            rospy.logwarn('The object fell during manipulation phase')
                        else:
                            lift_height_after_manipulation = height_after_tilt_attempt - self.object_height_after_grasp
                            if lift_height_after_manipulation >= 0.015: # make it adaptive (i.e. a certain percentage of the upward movement) and maybe take into account std to evaluate how 'sure' the measure is
                                rospy.loginfo('Successful manipulation.')
                                self.episode_results['large_tilt'] = True
                            else:
                                rospy.loginfo('Unsuccessful manipulation.')
                                self.episode_results['large_tilt'], self.episode_results['fall'], self.episode_results['when_fall_happened'] = False, True, 'large_tilt'
                    else:
                        rospy.logwarn('The marker is not visible, the object is considered as fallen.')
                        self.episode_results['large_tilt'], self.episode_results['fall'], self.episode_results['when_fall_happened'] = False, True, 'large_tilt' # if the marker is not visible, consider the bottle as fallen
            else:
                rospy.logwarn('Tilt was not attempted and the reward/penalty for tilt are not going to be assigned.')

    def place(self):
        if (not self.episode_results['fall'] and self.episode_results['lift']) or not self.aruco:   # if the bottle has fallen or has not been lifted, just detach the object 
                                                                                                    # in the planning scene and open the hand
            # release the mrl joint to set the bottle upright
            move.move_joints(self.hand, joint_values = [4.0], joints_to_move = ['hand_mrl_joint'])
            # put the object down                                              
            move.straight_eef_movement(self.arm, direction = 'z', translation = -0.05, avoid_collisions = False)
        # release the grasp and detach the object
        self.playmotion.playmotion_movement('open', planning = False)           # release the grasp
        self.scene.remove_attached_object(link = 'hand_grasping_frame')
        rospy.sleep(2) #[s] without this, the following instruction is skipped
        if (not self.episode_results['fall'] and self.episode_results['lift']) or not self.aruco: # if the bottle has fallen or has not been lifted, just plan to the homing position
            # move the hand forward
            move.straight_eef_movement(self.arm, direction = 'x', translation = -0.02, avoid_collisions = False)
            rospy.sleep(1)
            move.straight_eef_movement(self.arm, direction = 'y', translation = -0.08, avoid_collisions = False)
            # re-add the bottle model
            bottle_pose = PoseStamped()
            bottle_pose.header.frame_id = 'base_footprint'
            bottle_pose.pose.position = Point(*self.episode_results['center'])
            bottle_pose.pose.orientation = Quaternion(0, 0, 0, 1)
            self.scene.add_cylinder('bottle',                                                              # name
                                    bottle_pose,     # pose
                                    height = self.episode_results['height'], radius = self.episode_results['radius'])
            if self.pictures:
                # take a pic(?) why not?
                self.take_a_pic(name = 'place')
            if self.aruco: # if the bottle is already fallen, do not further check for its position
                # where is the marker?
                misalignment_place, height_after_place_attempt = self.aruco_orientation(self.object_pose_initial, time_limit = self.TIME_LIMIT)
                if height_after_place_attempt:
                    if height_after_place_attempt < self.bottle_top:
                        self.episode_results['fall'], self.episode_results['when_fall_happened'] = True, 'place'
                        self.episode_results['misalignment_place'] = round(misalignment_place, 4)
                        rospy.logwarn('The object fell during place phase')
                else:
                    rospy.logwarn('The marker is not visible, the object is considered as fallen.')
                    self.episode_results['fall'], self.episode_results['when_fall_happened'] = True, 'place' # if the marker is not visible, consider the bottle as fallen

    def save_results(self):
        # save the csv with the results to a file
        csv_filepath = os.path.join(self.path_to_package, 'saved', self.today_string + '.csv')
        file_already_exists = os.path.isfile(csv_filepath)
        rospy.loginfo('Writing to csv located at ' + csv_filepath)
        with open(csv_filepath, 'a') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames = self.episode_results.keys())
            if not file_already_exists:
                writer.writeheader()
            writer.writerow(self.episode_results)

    def reset(self):
        ##########
        # Remove the attached object from the grasping frame, clear the planning scene from said object. Then, based on user
        # input, either tuck the arm or just move the end-effector some cm away. Finally, remove the objects from the planning
        # scene and clear the octomap if it is requested.
        ##########
        # go to a safe position
        self.scene.remove_attached_object(link = 'hand_grasping_frame') # in case the object is still attached
        move.move_joints(self.arm_torso, joint_values = [0.30], joints_to_move = ['torso_lift_joint']) #self.extend_torso(elevation = 0.35)
        # perform one of the possible exit strategies depending on the corresponding keyvalue in the configuration .yaml file
        if self.current_escape == 'nearby':
            rospy.loginfo('Planning to reach the \'nearby\' homing position.')
            goal_pose = self.arm.get_current_pose(end_effector_link = 'hand_grasping_frame')
            if goal_pose.header.frame_id == 'base_footprint':
                goal_pose.pose.position.y -= 0.3
                goal_pose.pose.position.z += 0.1
                goal_pose.pose.orientation = Quaternion(0, 0, 0, 1)
                success, __ = move.eef_to_valid_pose(self.arm, goal_pose.pose.position, goal_pose.pose.orientation, exploration_direction = 'y', shift_amount = 0.05, max_shift = 0.10)
                if success:
                    rospy.loginfo('The \'nearby\' homing position has been successfully reached.')
                else:
                    rospy.logwarn('Impossible to reach the \'nearby\' homing position.')
            else:
                rospy.logerr('The current pose is in the ' + goal_pose.header.frame_id + '. The \'nearby\' homing is meant to work in the \'base_footprint\' frame, hence it won\'t be executed.')
        elif self.current_escape == 'home':
            # bring the arm far from the table
            move.execute_remembered_joint_values(self.arm, 'open_wide_right')
            # restore the collision matrix in which collisions with the hand safety box are forbidden
            self.collision_manager.clear_collision_matrix_default_entries() # seems to have no effect
            self.arm.clear_path_constraints()   # get rid of constraints
            # bring the arm close to the robot body
            self.playmotion.playmotion_movement('home', timeout = 20) # be careful: this is pre-recorded
        else:
            rospy.logwarn('No valid exit strategy was given, the robot is going to stay there.')
        # clear the octomap from objects
        self.scene.remove_world_object('bottle')
        if self.aruco:
            self.scene.remove_world_object('aruco')

        if self.clear_all:
            rospy.loginfo('Clearing the octomap.')
            rospy.wait_for_service('/clear_octomap')                        # keeping the octomap for a certain number of iteration,
            clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)     # (in a controlled environment) would spare some time
            clear_octomap()                                                 # without affecting the safety

    def reward_shaping(self): # called when iterating the 'task' within a learning process: this method combines the task outcomes and outputs a suitable reward (dimensionality is chosen by the user)
        # if one misalignment is missing, leave it as it is, but give a penalty as it was 90 degrees
        # consider moving from None to float('NaN') 
        # rewards history:  1. lift * 100 - misalignment_lift * 10 - misalignment_place * 10
        #                   2. lift * 100 - misalignment_tilt * 10 + 50 if manipulation - 50 if fall
        #                   3. today                    
        if self.episode_results['lift'] is not None:
            reward = 100 if self.episode_results['lift'] else -100 # 100 if lifted, -100 if not
            if self.episode_results['lift']: # only if the lift was good
                if self.episode_results['small_tilt']:
                    reward += 50
                if self.episode_results['large_tilt']:
                    reward += 100
                    reward = reward - 50 if self.episode_results['fall'] else reward + 50 
        else:
            reward = None
        return reward

def task(joint_values_for_grasp = [], grasp_height_delta = None):
    grasper = Grasping()
    try:
        grasp_point = grasper.build_scene('bottle') # look for the object and for possible obstacles
        
        # fix the grasp height
        grasp_height_delta = grasper.grasp_height_delta if grasper.grasp_height_delta is not None else grasp_height_delta
        if grasp_height_delta is not None:
            tf_displacement_wrt_hand_palm_extremes = 0.035 # the hand palm as a width of 7cm, the 'hand_grasping_frame' is placed at half width, close to the fingers
            desired_grasp_height = grasper.table_height + BOTTLE_MIN_RADIUS_HEIGHT + grasp_height_delta - tf_displacement_wrt_hand_palm_extremes
            grasp_point.z = desired_grasp_height
        # set the grasping point (the rightmost point of the object at half height) and the grasp pose
        actual_grasp_point = grasper.approach_object(grasp_point = grasp_point,
                                                    grasp_orientation = Quaternion(*grasper.eef_orientation))
        if grasper.episode_results['fall']:
            rospy.logerr('The object fell during hand approach! The proper exit stategy will be performed and the task ended.')
        else:
            # check the grasp height after the approach
            rospy.loginfo('Difference wrt the desired grasp height: ' + str(abs(desired_grasp_height - actual_grasp_point.z)))
            if grasp_height_delta is not None and abs(desired_grasp_height - actual_grasp_point.z) > 0.005:
                move.straight_eef_movement(grasper.arm, direction = 'z', translation = desired_grasp_height - actual_grasp_point.z, velocity_downscaling = 2.0, avoid_collisions = False)
            grasper.grasp_object('bottle', joint_values = joint_values_for_grasp) # grasp the object following the given grasp strategy
            # once the hand is closed, try to lift the object and then put it down
            rospy.logwarn('Grasp finished, going to lift.')
            grasper.lift()
            if grasper.episode_results['lift'] or not grasper.aruco: # grasper.episode_results['lift'] is None:
                grasper.manipulation()
            
            grasper.place() # place is needed even if the lift fails to detach the object from the grasping frame            
    
    except PointcloudException: # exceptions handling
        rospy.logerr('No valid pointcloud, check that you have launched the nodes for object segmentation and that the object to grasp is in the robot field of view.')
    except ArucoException:
        rospy.logerr('It was not possible to find any ArUco marker when building the scene. The reward cannot be assigned without it.')
    except PlanningException:
        rospy.logerr("It was not possible to find a valid grasping position.")
    finally: # the exit strategy is performed even in case of failure of the task
        grasper.reset()  # bring the robot back to a safe rest position and remove undesired elements from the planning scene
        if grasper.save:
            grasper.save_results()
        if grasper.learning:
            return grasper.reward_shaping()
    
    '''
    # before switching off the real robot, if needed, comment the whole main but these lines
    # ATTENTION:   - move TIAGo far from objects
    #              - if needed bring the arm far from the torso exploiting the gravity-compensation demo
    rospy.init_node('pickup')
    grasper = Grasping()
    grasper.playmotion.playmotion_movement('home', planning = False) 
    '''

if __name__ == '__main__':
    try:
        rospy.init_node('pickup')
        task()
    except KeyboardInterrupt:
        rospy.logwarn('Shutting down the grasping node')