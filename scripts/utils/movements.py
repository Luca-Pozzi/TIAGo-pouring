#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_commander.exception import MoveItCommanderException 
from sensor_msgs.msg import JointState

import copy

def execute_remembered_joint_values(group, name, velocity_scaling = 0.2):
    ##########
    # Plan and execute a trajectory to bring the arm to a remembered joint value
    ##########
    if not name in group.get_remembered_joint_values():
        rospy.logerr('\'' + name + '\' is not a remembered joint value.')
    else:
        group.set_max_velocity_scaling_factor(velocity_scaling)
        plan = group.plan(name)
        if plan.joint_trajectory.points: # True if trajectory contains points
            group.execute(plan)
        else:
            rospy.logwarn('The \'' + name +'\' joint value cannot be safely reached now.')

def straight_eef_movement(group, direction, translation, eef_step = 0.005, jump_threshold = 0.0, avoid_collisions = True):
    ##########
    # Move the end-effector along a straight line in the specified direction
    # direction:            [string] must be 'x, 'y' or 'z'
    # translation:          [float] distance in [m] of the end-effector translation
    # avoid_collisions:     [bool] if set to True, moves the end-effector even if a collision is detected
    ##########
    waypoints = []
    # add a waypoint
    waypoint_pose = group.get_current_pose().pose
    if direction == 'x':
        waypoint_pose.position.x += translation 
    elif direction == 'y':
        waypoint_pose.position.y += translation
    elif direction == 'z':
        waypoint_pose.position.z += translation
    waypoints.append(copy.deepcopy(waypoint_pose))       
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,       # waypoints to follow
                                       eef_step,        # interpolation resolution in [m]
                                       jump_threshold,  # jump_threshold (0.0 means it is disabled)
                                       avoid_collisions = avoid_collisions)
    # add velocity control/scaling       
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
        pass  
    group.go()
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
    return absolute_errors, relative_errors # read them when looking 