#!/usr/bin/env python
# make the robot grasp a certain object

# ROS libs
#import tf
import rospy
import moveit_commander 
# ROS messages
#from segmentation.msg import ObjCloud
#from std_srvs.srv import Empty
import ros_modules.moveit_modules.movements as move
from geometry_msgs.msg import Quaternion, PointStamped, Pose, Point, PoseStamped
from moveit_msgs.msg import RobotTrajectory 
from trajectory_msgs.msg import JointTrajectoryPoint
# Python libs
import sys, os, math


class MoveItTest:
    def __init__(self):
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
        rospy.sleep(2)
        self.arm.remember_joint_values('pre_tilt_pose', values = [math.pi/3, 0.0, -math.pi/2, math.pi/3, -math.pi/2, -math.pi/6, 0.0])

    def test_planning(self):
        target = PoseStamped()
        target.header.frame_id = 'base_footprint'
        target.pose.position = Point(0.86, -0.05, 0.90)
        target.pose.orientation = Quaternion(0.707, 0, 0, 0.707)
        self.arm.set_pose_target(target)
        self.arm.execute(self.arm.plan())

    def extend_arm(self):
        #self.arm.clear_path_constraints()
        move.execute_remembered_joint_values(self.arm, 'pre_tilt_pose')     # stretch the arm in front of the robot

    def pour(self):
        tilt_deg = 1.745 # = 100 degrees to see if the liquid in the bottle could be poured while holding it
        # tilt the wrist
        success = False
        initial_joint_value = self.arm.get_current_joint_values()[-1]
        rospy.loginfo('Wrist joint value is ' + str(initial_joint_value)) 
        final_joint_value = initial_joint_value - tilt_deg
        #if final_joint_value > -2.2:
        move.move_joints(self.arm, joint_values = [final_joint_value], joints_to_move = ['arm_7_joint'])
        rospy.sleep(1)
        # back to the initial pose
        move.move_joints(self.arm, joint_values = [initial_joint_value], joints_to_move = ['arm_7_joint'])

if __name__ == '__main__':
    rospy.init_node('moveit_test')
    moveit_tester = MoveItTest()
    try:
        moveit_tester.extend_arm()
        moveit_tester.pour()
    except KeyboardInterrupt:
        rospy.logwarn('Shutting down the grasping node')