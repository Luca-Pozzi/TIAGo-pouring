#!/usr/bin/env python
# wave and then move to a predefined position

# ROS libs
#import tf
import rospy #, rospkg
import moveit_commander 
# ROS messages
#from segmentation.msg import ObjCloud
#from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion, PointStamped, Pose, Point, PoseStamped, Vector3, Vector3Stamped
#from sensor_msgs.msg import CompressedImage
#from moveit_msgs.msg import RobotTrajectory 
#from trajectory_msgs.msg import JointTrajectoryPoint
# Python libs
import sys, os
import cv2
#import datetime
#import yaml, csv
import numpy as np
import math
# my scripts
#import geometry.points as pts
#import geometry.quaternions as quat
#from ros_modules import roslauncher
from ros_modules.Playmotion import Playmotion
import ros_modules.moveit_modules.movements as move
#from ros_modules.moveit_modules.planning_scene import CollisionManager, ConstraintManager, load_moveit_config
#from utils.errors import PointcloudException, ArucoException, PlanningException

if __name__ == '__main__':
    try:
        rospy.init_node('inauguration')
        playmotion = Playmotion()
        playmotion.playmotion_movement('wave', timeout = 15)

        # initialize a moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        # exploit MoveIt to move TIAGo
        arm = moveit_commander.MoveGroupCommander('arm')
        arm.set_max_velocity_scaling_factor(0.2)
        rospy.loginfo('MoveIt commander initialized.\n')

        arm.remember_joint_values('get_ready', values = [0.52, 0.29, -0.77, 1.21, 0.41, 0.66, -1.37])
        move.execute_remembered_joint_values(arm, 'get_ready')

    except KeyboardInterrupt:
        rospy.logwarn('Shutting down the grasping node')