#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_commander.exception import MoveItCommanderException 
#import moveit_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

#import copy

def execute_remembered_joint_values(group, name):
    ##########
    # Plan and execute a trajectory to bring the arm to a remembered joint value
    ##########
    if not name in group.get_remembered_joint_values():
        rospy.logerr('\'' + name + '\' is not a remembered joint value.')
    else:
        plan = group.plan(name)
        if plan.joint_trajectory.points: # True if trajectory contains points
            group.execute(plan)
        else:
            rospy.logwarn('The \'' + name +'\' joint value cannot be safely reached now.')

def straight_eef_movement(group, direction, translation, eef_step = 0.005, jump_threshold = 0.0, avoid_collisions = True, constraints = None, velocity_downscaling = 1.0):
    ##########
    # Move the end-effector along a straight line in the specified direction
    # direction:            [string] must be 'x, 'y' or 'z'
    # translation:          [float] distance in [m] of the end-effector translation
    # avoid_collisions:     [bool] if set to True, moves the end-effector even if a collision is detected
    # velocity_downscaling: [float] the velocity of the movement will be slowed by this factor (there is no control on absolute velocity of the movement)
    ##########
    #waypoints = []     # give the possibility to specify a "composed" trajectory with an input string like '-5y 1x' representing a movement of 5cm toward right and 1cm forward
    # add a waypoint
    goal_point_pose = group.get_current_pose().pose
    if direction == 'x':
        goal_point_pose.position.x += translation 
    elif direction == 'y':
        goal_point_pose.position.y += translation
    elif direction == 'z':
        goal_point_pose.position.z += translation
    #if constraints:     
    (plan, fraction) = group.compute_cartesian_path(
                                       [goal_point_pose],   # waypoints to follow
                                       eef_step,            # interpolation resolution in [m]
                                       jump_threshold,      # jump_threshold (0.0 means it is disabled)
                                       avoid_collisions = avoid_collisions,
                                       path_constraints = constraints)
    '''
    else:
        (plan, fraction) = group.compute_cartesian_path(
                                           [goal_point_pose],   # waypoints to follow
                                           eef_step,            # interpolation resolution in [m]
                                           jump_threshold,      # jump_threshold (0.0 means it is disabled)
                                           avoid_collisions = avoid_collisions)
    '''
    ## velocity scaling (just relative, get the eef_joint and compute its displacement to compute (and eventually impose) an absolute velocity)
    if not velocity_downscaling == 1.0:
        for idx, point in enumerate(plan.joint_trajectory.points):
            plan.joint_trajectory.points[idx].time_from_start = point.time_from_start * velocity_downscaling

    if plan.joint_trajectory.points:
        group.execute(plan)  # move the robot
        group.stop() # make sure there is no residual movement
    rospy.loginfo(str(int(fraction * 100)) + '% of the end-effector translation of ' + str(translation) + 'm along the ' + direction + ' direction has been completed.')

def move_joints(group, joint_values = [], joints_to_move = []):
    ##########
    # Bring the hand to the desired configuration. A sequence of configuration can be given to make the hand perform
    # a sequence of movements.
    # postures:     [[float]] joint values [rad]
    # positions:    [[float]] joint positions. 
    #               Measurement unit is unclear, maximum value are [6.8, 9.2, 6.2] for [index mrl thumb]
    # TO ADD: velocities [[float]] maybe even relative to the default ones
    # joint_names:  [[string]] joint names
    ##########
    rospy.loginfo('Preparing to move ' + str(joints_to_move) + ' to ' + str(joint_values))
    joint_names = group.get_active_joints()

    target_joint_values = group.get_current_joint_values()
    for idx, joint_name in enumerate(joint_names):
        if joint_name in joints_to_move:
            target_joint_values[idx] = joint_values[joints_to_move.index(joint_name)]
    try:
        # the following line throws an unexpected exception "Is the target within bounds?", now it runs smoothly after the catching the error
        # as suggested at https://github.com/Kinovarobotics/kinova-ros/issues/110
        group.set_joint_value_target(target_joint_values)
    except MoveItCommanderException as e:
        rospy.logwarn('MoveIt raised an exception')
    execution_success = group.go()
    group.stop()
    # check if the movement has been performed as expected
    current_joint_values = group.get_current_joint_values()
    
    # compute the error (absolute and relative) for each joint
    absolute_errors = [current_joint_value - target_joint_value for current_joint_value, target_joint_value in zip(current_joint_values, target_joint_values)]
    relative_errors = [(current_joint_value - target_joint_value) / target_joint_value for current_joint_value, target_joint_value in zip(current_joint_values, target_joint_values)]
    ''' consider printing the current value of the joints_to_move and let the user specify a threshold (either abs or rel) to establish if the movement has been accomplished
    for idx, joint_name in enumerate(joint_names):
        if joint_name in joints_to_move:
            target_joint_values[idx] = joint_values[joints_to_move.index(joint_name)]
    '''
    return execution_success, absolute_errors, relative_errors

''' not working
def eef_to_pose(group, eef_target_position, eef_target_orientation, max_attempts = 10):
    ##########
    # Bring the end-effector to the given pose.
    # eef_target_position:      [Point] the point to which the hand_grasping_frame has to be positioned
    # eef_target_orientation:   [Quaternion] the orientation of the hand in the final point
    # shift_amount:             [float] if the eef_target_position cannot be reached, the target point is raised
    #                           of shift_amount [m] until a valid position is reached
    # max_attempts:             [int] the maximum number of planning attempts
    ##########
    # get ready for the while loop
    planning_success = False
    execution_success = False
    attempt = 1
    eef_target = Pose(eef_target_position, eef_target_orientation)
    while attempt <= max_attempts and not planning_success:  # try to grasp the bottle at the eef_target_position, if it is not possible, retry a given number of times
        rospy.loginfo('Planning attempt #' + str(attempt))
        group.set_pose_target(eef_target)
        # plan and execute the action
        plan = group.plan()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            # check if there are non-positive time steps and remove them (became necessary when introducing the 'elbow' contraint on TIAGo)
            waypoints_before_check = len(plan.joint_trajectory.points[:-1])
            non_positive_time_steps_indexes = [idx for idx, point in enumerate(plan.joint_trajectory.points[:-1]) if int(str((plan.joint_trajectory.points[idx + 1].time_from_start - plan.joint_trajectory.points[idx].time_from_start))) <= 0]
            if non_positive_time_steps_indexes:
                for idx in non_positive_time_steps_indexes:
                    plan.joint_trajectory.points.pop(idx)
                rospy.logwarn('Removed {removed_points} out of {waypoints} waypoints not increasing in time from the planned trajectory'.format(removed_points = len(non_positive_time_steps_indexes),
                                                                                                                            waypoints = waypoints_before_check))  
            planning_success = True
        else:
            rospy.logwarn('No valid plan found, retrying.')
            attempt += 1
    # if a valid plan has been found, try to execute it
    if planning_success:
        rospy.loginfo("Bringing the end-effector to:\n" + str(eef_target))
        execution_success = group.execute(plan)
        group.stop()  # be sure that there is no residual movement
    else:
        rospy.logerr('Impossible to reach given pose.') # format and print the actual pose

    return execution_success, eef_target_position
'''

def eef_to_valid_pose(group, eef_target_position, eef_target_orientation, exploration_direction = 'z', shift_amount = 0.01, max_shift = 0.10):
    ##########
    # Bring the end-effector to the given pose.
    # eef_target_position:      [Point] the point to which the hand_grasping_frame has to be positioned
    # eef_target_orientation:   [Quaternion] the orientation of the hand in the final point
    # shift_amount:             [float] if the eef_target_position cannot be reached, the target point is raised
    #                           of shift_amount [m] until a valid position is reached
    # max_shift:                [float] a limit (in [m]) to the overall shift
    ##########
    # get ready for the while loop
    planning_success = False
    execution_success = False
    #valid_height_value = True
    total_shift = 0.0
    eef_target = Pose(eef_target_position, eef_target_orientation)
    while (total_shift <= max_shift) and not planning_success:  # try to grasp the bottle at the eef_target_position, if it is not
                                                       # possible, raise the grasping point until a reasonable limit
        group.set_pose_target(eef_target)
        # plan and execute the action
        plan = group.plan()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            # check if there are non-positive time steps and remove them (became necessary when introducing the 'elbow' contraint on TIAGo)
            waypoints_before_check = len(plan.joint_trajectory.points[:-1])
            non_positive_time_steps_indexes = [idx for idx, point in enumerate(plan.joint_trajectory.points[:-1]) if int(str((plan.joint_trajectory.points[idx + 1].time_from_start - plan.joint_trajectory.points[idx].time_from_start))) <= 0]
            if non_positive_time_steps_indexes:
                for idx in non_positive_time_steps_indexes:
                    plan.joint_trajectory.points.pop(idx)
                rospy.logwarn('Removed {removed_points} out of {waypoints} waypoints not increasing in time from the planned trajectory'.format(removed_points = len(non_positive_time_steps_indexes),
                                                                                                                            waypoints = waypoints_before_check))  
            planning_success = True
        else:
            rospy.logwarn('Try planning again with different target. The grasping target is shifted ' + str(shift_amount) + ' above with respect to the original.')   
            # shift the target point until a valid one its found
            if exploration_direction == 'z':
                eef_target.position.z += shift_amount
            if exploration_direction == 'y':
                eef_target.position.y += shift_amount
            if exploration_direction == 'x':
                eef_target.position.x += shift_amount
            # track the total shift wrt the initial point
            total_shift += shift_amount
            rospy.loginfo(eef_target)
    # if a valid plan has been found, try to execute it
    if planning_success:
        rospy.loginfo("Bringing the end-effector to:\n" + str(eef_target))
        execution_success = group.execute(plan)
        group.stop()  # be sure that there is no residual movement

    return execution_success, eef_target.position

'''
def plan_to_joint_state(group, positions = [], joint_names = []): # redundant, same effect of move_joints
    ##########
    # Bring the hand to the desired configuration. A sequence of configuration can be given to make the hand perform
    # a sequence of movements.
    # postures:     [[float]] joint values [rad]
    # positions:    [[float]] joint positions. 
    #               Measurement unit is unclear, maximum value are [6.8, 9.2, 6.2] for [index mrl thumb]
    # joint_names:  [[string]] joint names
    ##########
    if not joint_names:
        joint_names = group.get_active_joints()
    # check that the number of joint_names is equal to the number of elements in a single position
    postures = [JointState(name = joint_names, position = position) for position in positions]
    # let the robot apply the selected postures
    for posture in postures: 
        group.set_joint_value_target(posture)
        plan = group.plan()
        if plan.joint_trajectory.points: # True if trajectory contains points
            group.execute(plan)
        else:
            rospy.logwarn("Cannot move the fingers in the given posture.")
'''