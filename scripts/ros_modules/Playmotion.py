#!/usr/bin/env python
# make the robot perform a pre-defined motion

# ROS libs
import rospy
# ROS messages
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

class Playmotion:
    def __init__(self):
        ### ActionClient
        # create an action client for PlayMotionAction
        self.playmotion = SimpleActionClient("play_motion", PlayMotionAction)
        self.playmotion.wait_for_server() # wait until the action sever has started

    def playmotion_movement(self, movement, planning = True, timeout = 10.0):
        ##########
        # Play a pre-recorded movement exploiting the playmotion package.
        # movement:     [string] the name of a playmotion movement. 
        #               Check http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/play_motion for a list of movement.
        #               More movements can be executed in this way if the proper parameters are loaded.
        # planning:     [bool] wheter to plan or not the movement toward the first point of the pre-recorded trajectory.
        #               NOTE: during the execution of the movement NO PLANNING is done.
        # timeout:      [float] maximum time in [s] for the action execution.
        ##########
        rospy.loginfo('Preparing to play the \'' + movement + '\' movement.')
        # exploit a pre-defined PlayMotion
        goal = PlayMotionGoal()
        goal.motion_name = movement
        goal.skip_planning = not planning
        # execute the movement
        self.playmotion.send_goal(goal)
        success = self.playmotion.wait_for_result(rospy.Duration(timeout))
        if success:
            rospy.loginfo('The \'' + movement + '\' movement has been executed.')
        else:
            rospy.logwarn('Timeout (' + str(timeout) + 's) elapsed. The \'' + movement + '\' movement has not been completed.')