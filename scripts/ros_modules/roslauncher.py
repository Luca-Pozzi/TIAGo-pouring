#!/usr/bin/env python
# simple functions to programatically launch nodes, eploiting the roslaunch API
# http://wiki.ros.org/roslaunch/API%20Usage

# ROS libs
import rospy, roslaunch

def roslaunch_from_file(path, args = []):
    # programatically launch the nodes for vision
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    if args:
        # resolve arguments and instantiate a ROSLaunchParent object
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments([path] + args)[0], args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    else:
        # instantiate a ROSLaunchParent object    
        parent = roslaunch.parent.ROSLaunchParent(uuid, [path])
    parent.start() # launch the nodes
    return parent #returned in order to be available to shutdown the node

''' it works, but using the web commander to stop 'head_manager' results in a smoother shutdown
def keep_head_still(self): 
    ##########
    # When working with the real robot, the /pal_head_manager node is launched at startup. It implements people tracking
    # which is not needed during grasping and could cause the robot to move its head away from the target. This method
    # checks if such node is running and if it is, shuts it down.
    ##########
    # check if the /pal_head_manager node is running and kill it to keep TIAGo's head still
    node_name = '/pal_head_manager' # check the node name
    is_running = rosnode.rosnode_ping(node_name, max_count = 10)
    print("Is the node " + node_name + " running?" + str(is_running))
    if is_running:
        # if the node is running, shut it down
        rosnode.kill_nodes([node_name])
        # check that the node is no more running
        print("And now? " + str(rosnode.rosnode_ping(node_name, max_count = 10)))
    # if this method is considered likely to fail, it could be included in a while loop
'''