#!/usr/bin/env python

# wrap the TIAGo grasping task within a bo_ros evalutation node

# ROS libs
import rospy
# ROS messages
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
# Python libs
import numpy as np
import yaml
# my scripts
from Grasping import task

def multiarray_msg_to_numpy_ndarray(msg):
    return np.array(msg.data).reshape((msg.layout.dim[0].size, msg.layout.dim[1].size))

def format_multiarray(data, labels = ['samples_list', 'sample']):
    if len(data.shape) == 2:
        cardinality = data.shape[1]
    elif len(data.shape) == 1:
        cardinality = 1
    else:
        pass #raise an error
    number_of_points = data.shape[0]
    # format the MultiArray message
    msg = Float64MultiArray()
    msg.data = data.flatten()
    # set the array layout
    dim = [MultiArrayDimension() for i in range(2)]
    dim[0].label = labels[0]
    dim[0].size = number_of_points 
    dim[0].stride = cardinality * number_of_points
    dim[1].label = labels[1]
    dim[1].size = cardinality
    dim[1].stride = cardinality
    msg.layout.dim = dim
    return msg # return the formatted message

class Eval:
    def __init__(self):
        self.x_sample_subscriber = rospy.Subscriber('/bo/x_next', Float64MultiArray, self.evaluate)
        self.y_eval_publisher = rospy.Publisher('/bo/y_next', Float64MultiArray, queue_size = 1)
        with open(rospy.get_param('bo/config')) as file:
            self.config = yaml.load(file.read())
            
    def evaluate(self, X_msg): # black box function returning evaluations of the specified points
        X = multiarray_msg_to_numpy_ndarray(X_msg)

        # the core of the evaluation function
        points_eval = []
        for x in X:
            rospy.loginfo('Evaluating the grasping procedure with: ' + str(x))
            reward = task(joint_values_for_grasp = x[0:3].tolist(), grasp_height_delta = x[3])
            while reward is None: # if lift is None (i.e. not attempted, retry)
                rospy.logwarn('Lift was not attempted, the same grasp configuration will be tried again.')
                reward = task(joint_values_for_grasp = x[0:3].tolist(), grasp_height_delta = x[3])
            # the BO is set up as a minimization task hence negative values are good. The reward shaping is done considering positive rewards
            # and negative penalties to match the RL paradigm
            rospy.loginfo('Reward sent to BO: ' + str(-reward))
            points_eval.append(-reward) 
        while not self.y_eval_publisher.get_num_connections():
            pass # wait for a node to connect to the published topic
        # publish the message instead
        self.y_eval_publisher.publish(format_multiarray(np.array(points_eval)))

if __name__ == '__main__':
    # initialize the ROS node
    rospy.init_node('eval') # anonymous = True
    evaluation = Eval()
    try:
        rospy.spin()
    except KeyboardInterrupt: # shutdown on user request
        print("Shutting down.")