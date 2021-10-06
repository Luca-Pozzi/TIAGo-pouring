#!/usr/bin/env python
# the MarkerVisualization class provides methods to manage the visualization of points and vectors in Rviz
# the CollisionManager class allows to enable/disable collisions in the current MoveIt! planning scene

# ROS libs
import tf
import rospy
import moveit_commander
# ROS messages
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningSceneComponents, PlanningScene, AllowedCollisionEntry, AllowedCollisionMatrix, Constraints, PositionConstraint, BoundingVolume, VisibilityConstraint, JointConstraint, OrientationConstraint
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, Vector3
from shape_msgs.msg import SolidPrimitive
# Python libs
import copy
from math import pi

def load_moveit_config(group, config_dict):
    for key in config_dict:
        # this exaustive manual listing is not great, but it is a nice 'quick-and-dirty' solution
        if key == 'goal_joint_tolerance':
            group.set_goal_joint_tolerance(config_dict['goal_joint_tolerance'])
        elif key == 'goal_position_tolerance':
            group.set_goal_position_tolerance(config_dict['goal_position_tolerance'])
        elif key == 'goal_tolerance':
            group.set_goal_joint_tolerance(config_dict['goal_tolerance'])
        elif key == 'max_acceleration_scaling_factor':
            group.set_max_acceleration_scaling_factor(config_dict['max_acceleration_scaling_factor'])
        elif key == 'max_velocity_scaling_factor':
            group.set_max_velocity_scaling_factor(config_dict['max_velocity_scaling_factor'])
        elif key == 'num_planning_attempts':
            group.set_num_planning_attempts(config_dict['num_planning_attempts'])
        elif key == 'planning_time':
            group.set_planning_time(config_dict['planning_time'])
        else:
            rospy.logwarn('Key named \'' + key + '\' is not valid and therefore cannot be set.') 

class MarkerVisualization:
    def __init__(self):
        # initializer a publisher for Marker messages
        pass

    def display_arrow(self):
        pass

class ConstraintManager:
    def __init__(self):
        pass

    def set_constraints(self, centroid, keyvalues, visibility = True, position = True):
        ##########
        # Set some pre-defined constraint make the robot grasp the object from the right in a controlled way.
        # Unconstrained path-planning works, but is more likely to have unpredicted behaviors and since the
        # eef positioning is not part of the learned gestures, it should influence the final outcome as little
        # as possible.
        # centroid:     [PoseStamped] the centroid of the object to grasp
        # keyvalues:    [namedtuple] the most relevant coordinates of the object
        ##########
        constraint = Constraints()
        constraint.name = 'grasp_from_right'    # it makes sense since the TIAGo's arm is a right arm
        constraint.joint_constraints = []
        constraint.position_constraints = []
        constraint.orientation_constraints = []
        constraint.visibility_constraints = []
    
        if visibility:
            '''
            vis_constraint = VisibilityConstraint()
            # set target dimension, position and orientation
            vis_constraint.target_radius = (keyvalues.ymax - keyvalues.ymin) / 2
            vis_constraint.target_pose = PoseStamped(centroid.header,
                                                    Pose(Point(centroid.point.x, centroid.point.y, keyvalues.zmin), 
                                                        Quaternion(0, 0, 0, 1)))
            vis_constraint.cone_sides = 3   # minimum allowed, should be also the less strict
            # set sensor position and orientation
            sensor_pose = PoseStamped()
            sensor_pose.header.frame_id = 'xtion_rgb_optical_frame'
            sensor_pose.pose = Pose(Point(0, 0, 0.05),      # the point of the pose must be outside the robot
                                    Quaternion(0, 0, 0, 1))
            vis_constraint.sensor_pose = sensor_pose
            vis_constraint.sensor_view_direction = 0 # SENSOR_Z
            # do not enforce view angle
            vis_constraint.max_view_angle = 0
            vis_constraint. max_range_angle = 0
            # set a weight for the current constraint
            vis_constraint.weight = 1.0
            # add the visibility_constraint message to the constraint message
            constraint.visibility_constraints += [vis_constraint]
            '''
            vis_constraint = VisibilityConstraint()
            # set the target dimension, position and orientation
            vis_constraint.target_radius = 0.03
            vis_constraint.target_pose = PoseStamped(centroid.header,
                                                    Pose(Point(centroid.point.x, centroid.point.y, keyvalues.zmin + height + 0.03), # the disk is above the bottle, where the markers are
                                                        Quaternion(0, 0, 0, 1)))
            vis_constraint.cone_sides = 3   # minimum allowed, should be also the less strict
            # set sensor position and orientation
            sensor_pose = PoseStamped()
            sensor_pose.header.frame_id = 'xtion_rgb_optical_frame'
            sensor_pose.pose = Pose(Point(0, 0, 0.05),      # the point of the pose must be outside the robot
                                    Quaternion(0, 0, 0, 1))
            vis_constraint.sensor_pose = sensor_pose
            vis_constraint.sensor_view_direction = 0 # SENSOR_Z
            # do not enforce view angle
            vis_constraint.max_view_angle = 0
            vis_constraint. max_range_angle = 0
            # set a weight for the current constraint
            vis_constraint.weight = 1.0
            # add the visibility_constraint message to the constraint message
            constraint.visibility_constraints += [vis_constraint]
        '''
        if position:
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = 'base_footprint'
            pos_constraint.link_name = 'hand_grasping_frame'
            #pos_constraint.target_point_offset = Vector3(0, 0, 0) 
            # define a volume in which the eef-link should move
            bounding_vol = BoundingVolume()
            box = SolidPrimitive()
            box.type = 1                        # BOX
            box.dimensions = [1.0, 1.5, 1.0]    # [BOX_X, BOX_Y, BOX_Z]
            bounding_vol.primitives = [box]
            box_pose = Pose(Point(0.5, keyvalues.ymax - 0.74, keyvalues.zmin + 0.49), Quaternion(0, 0, 0, 1))
            bounding_vol.primitive_poses = [box_pose]
            pos_constraint.constraint_region = bounding_vol
            pos_constraint.weight = 1.0
            constraint.position_constraints += [pos_constraint]
        '''
        '''
        if orientation:
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = 'base_footprint'
            orient_constraint.link_name = 'hand_grasping_frame'
            orient_constraint.orientation = Quaternion(0, 0, 0, 1)
            # keep the hand 
            orient_constraint.absolute_x_axis_tolerance = 0.05   # the lower the better, but it may cause the planner to fail
            orient_constraint.absolute_y_axis_tolerance = pi
            orient_constraint.absolute_z_axis_tolerance = pi
            orient_constraint.weight = 1.0
            # add the joint_constraint message to the constraint message
            constraint.orientation_constraints += [orient_constraint]
        '''

        if not visibility:  # if visibility constrain is too strict, use this as a surrogate (i.e. do not place the first link right in front of the robot)
            '''
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = 'arm_1_joint'
            # let the shoulder joint move only from 0 to 75 degrees
            joint_constraint.position = pi/4
            joint_constraint.tolerance_above = pi/4 #pi/6
            joint_constraint.tolerance_below = pi/4
            joint_constraint.weight = 1.0
            # add the joint_constraint message to the constraint message
            constraint.joint_constraints += [joint_constraint]
            '''
            
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = 'arm_3_joint'
            # let the elbow joint move only from -30 to -180 degrees
            joint_constraint.position = -pi/2
            joint_constraint.tolerance_above = pi/3
            joint_constraint.tolerance_below = pi/2
            joint_constraint.weight = 1.0
            # add the joint_constraint message to the constraint message
            constraint.joint_constraints += [joint_constraint]
            
            
        return constraint

class CollisionManager:
    def __init__(self):
        pass

    def set_planning_scene(self, scene):
        # exploit a service call to apply a planning scene
        rospy.wait_for_service('/apply_planning_scene', 10.0)
        apply_planning_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        apply_planning_scene(scene)

    def allow_collisions(self, object_name = 'object', allowed_touch_links = [], disable_collision_links = []):
        ##########
        # Call a service to get the current planning scene and extract the allowed collision matrix from it. Edit
        # the matrix so to allow collisions between the gripper links and the object. Call another service to apply
        # the new matrix collision to the planning scene.
        # NOTE: once it is no more needed to allow the collisions, the matrix should be set back to its original state.
        # object_name:              [string] name with which the object is identified in the planning scene
        # allowed_touch_link_list:  [[string]] the name of the joints that are allowed to collide with the object
        # disable_collision_links:  [[string]] the name of the joints that are allowed to collide with any other link.
        #                           DISCLAIMER: this argument has been added to disable the collisions with the dummy
        #                                       link "hand_safety_box", any other usage if discouraged since it may be
        #                                       harmful for the robot.
        ##########
        # from https://answers.ros.org/question/157716/obstacles-management-in-moveit/
        # prepare to get the current planning scene 
        rospy.wait_for_service('/get_planning_scene', 10.0)
        get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        # get the current allowed collision matrix
        request = PlanningSceneComponents(components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = get_planning_scene(request)
        # get the allowed collision matrix
        acm = response.scene.allowed_collision_matrix
        original_acm = copy.deepcopy(acm) # save the ACM before modifying it so to be able to restore it later
        if allowed_touch_links: # if there is at least one element in allowed_touch_links
            # allow collision with the object
            if not object_name in acm.entry_names:
                acm.entry_names += [object_name]
                acm.entry_values += [AllowedCollisionEntry()] 
            for link in acm.entry_names:
                idx = acm.entry_names.index(link)
                if link in allowed_touch_links:
                    # if the link is in the gripper, allow it to touch the object and vice versa
                    acm.entry_values[idx].enabled += [True]
                    acm.entry_values[acm.entry_names.index(object_name)].enabled += [True]
                else:
                    # if the link is not in the gripper, collisions are forbidden
                    acm.entry_values[idx].enabled += [False]
                    acm.entry_values[acm.entry_names.index(object_name)].enabled += [False]
            acm.entry_values[acm.entry_names.index(object_name)].enabled.pop() # drop the last element, since there is an extra False (why?)
        if disable_collision_links: # if there is at least one element in allowed_touch_links
            for link in disable_collision_links:
                if not link in acm.default_entry_names:
                    # add the link to the allowed collision matrix
                    acm.default_entry_names += [link]
                    acm.default_entry_values += [True]
        # create a differential planning scene with the desired collision matrix
        planning_scene_diff = PlanningScene(is_diff = True, allowed_collision_matrix = acm)
        self.set_planning_scene(planning_scene_diff)
        return original_acm

    def clear_collision_matrix_default_entries(self):
        ##########
        # Clear all the defaults entry in the allowed collision matrix.
        ##########
        acm = AllowedCollisionMatrix()  # define a collision matrix
        # set no default names and values
        acm.default_entry_names = []     
        acm.default_entry_values = []
        # create a differential planning scene with such a collision matrix
        planning_scene_diff = PlanningScene(is_diff = True, allowed_collision_matrix = acm)
        # exploit a service call to apply it
        self.set_planning_scene(planning_scene_diff)
