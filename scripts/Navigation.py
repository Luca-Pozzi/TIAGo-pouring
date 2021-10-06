#!/usr/bin/env python
# make the robot navigate toward a certain point.
# Navigation part adapted from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

# ROS libs
import tf
import rospy
import roslaunch
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, PointStamped
from segmentation.msg import Centroid, ObjCloud   # custom message

# Python libs
import numpy as np

# my scripts
import quaternion_utils as quat
 
MANIPULATION_FRAME_ID = 'base_footprint'                # frame in which the coordinates of the target are expressed for manipulation tasks
NAVIGATION_FRAME_ID = 'map'                             # frame in which the coordinates of the target are expressed for navigation
VISION_LAUNCH_FILE = "/home/luca/tiago_public_ws/src/segmentation/launch/coordinates.launch"

class Navigation:
    def __init__(self):
        # create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = SimpleActionClient('move_base',MoveBaseAction)
        # waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        # create a Transform Listener that subscribes to the /tf topic and allows to transform ROS messages
        # from one reference frame to another 
        self.tf_listener = tf.TransformListener(cache_time = rospy.Duration(10))
        print('SimpleActionClient and TransformListener initialized.')

    def get_target_coord(self, obj_class, to_frame):
        # programatically launch the nodes for vision
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        # launch the nodes
        vision = roslaunch.parent.ROSLaunchParent(uuid, [VISION_LAUNCH_FILE])
        vision.start()
        # subscribe to the "/segmentation_result/cloud/semantic", read a message and then unsubscribe
        target = rospy.wait_for_message("/segmentation_result/cloud/semantic", ObjCloud, timeout = None)
        # check if the centroid is referred to an object of the chosen class
        # and if in target.centroid there is any nan value (that would cause an error when setting it as goal)
        while not (target.obj_class == str(obj_class)) or np.isnan(target.centroid.point.x) or np.isnan(target.centroid.point.y) or np.isnan(target.centroid.point.z):
            # if necessary, wait for another message from the "/segmentation_result/cloud/semantic", unitl a valid one is received
            target = rospy.wait_for_message("/segmentation_result/cloud/semantic", ObjCloud, timeout = None)
        # bring the target coordinates into the desired frame if necessary   
        if not to_frame == target.centroid.header.frame_id:
            from_frame = target.centroid.header.frame_id # re-assign for the sake of clarity 
            # self.navigation_target = self.tf_listener.transformPoint(frame, target.centroid) # there is a timing issue    # TIMING ISSUE, currently worked-around
            # wait for the proper transform to become available and store it
            self.tf_listener.waitForTransform(to_frame, from_frame, time = rospy.Time.now(), timeout = rospy.Duration(5)) # time = rospy.Time()
            translation, rotation = self.tf_listener.lookupTransform(to_frame, from_frame, 
                                                                    self.tf_listener.getLatestCommonTime(to_frame, from_frame))
            # roto-translate the point and save it as an attribute
            target.centroid = quat.transform_point(to_frame, target.centroid, translation, rotation)
        self.navigation_target = target.centroid # PoseStamped
        print('Target acquired.')
        vision.shutdown()   # shut down the nodes for vision during navigation

    def get_close_to_object(self, obj_class):
        # get the coordinates of the centroid of the target object
        self.get_target_coord(obj_class, NAVIGATION_FRAME_ID)
        print('Navigate to:\n' + str(self.navigation_target) + '\n')
        #print(self.navigation_target)
        #print('\n')
        # create a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # set the goal of the navigation
        goal.target_pose.header.frame_id = self.navigation_target.header.frame_id # which is now NAVIGATION_FRAME_ID
        goal.target_pose.header.stamp = rospy.Time.now()
        # move right in front of the bottle, but stand back from the table
        goal.target_pose.pose.position.x = self.navigation_target.point.x - 0.6 # hard-coding the distance from the object is not the best solution
                                                                                # a loop with try/except statements in case the solution is not
                                                                                # feasible should be preferred 
        goal.target_pose.pose.position.y = self.navigation_target.point.y
        # no rotation of the mobile base frame wrt map frame
        goal.target_pose.pose.orientation.w = 1.0
        # send the goal to the action server.
        self.client.send_goal(goal)
        # waits for the server to finish performing the action.
        wait = self.client.wait_for_result()
        # if the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
        # result of executing the action
            return self.client.get_result()

if __name__ == '__main__':
    rospy.init_node('get_close')
    navigator = Navigation()
    # exploit the segmentation package to detect the object, retrieve its coordinates and get close to it
    navigator.get_close_to_object(39) # corresponding to "bottle" in the segmentation output