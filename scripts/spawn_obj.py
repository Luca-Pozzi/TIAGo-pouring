#!/usr/bin/env python
# spawn a table with a bottle and a cup on top of it in a random pose close to the world origin

# ROS libs
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose, Quaternion, Point

# Python libs
import random, tf, time

# .sdf files locations (not all of them are used)
TABLE_SDF_FILE = "/opt/pal/ferrum/share/tiago_gazebo/models/table_0m8/table.sdf"  #[0.80m]
CUP_SDF_FILE = "/root/tiago_public_ws/src/tiago_pouring/models/plastic_cup/model.sdf"
BOTTLE_SDF_FILE = "/root/tiago_public_ws/src/tiago_pouring/models/bottle3_full/model.sdf" #empty #third #half #full #full_small 

# geometric constants and params
TABLE_OFFSET = 0 #0.5		# [m] maximum distance along x and y directions with respect to the (TABLE_X, TABLE_Y) point
TABLE_ROTATION = 0 #3.14/8 # [rad] maximum rotation around the z axis (i.e. yaw angle)
OBJECTS_OFFSET = 0.1	# [m] maximum distance of the objects above the table from the table origin (i.e. center)
TABLE_HEIGHT = 0.82  	# [m] hardcoded value to spawn the objects above the table surface
TABLE_X = 4 			# [m] nominal x component of the table spawn point
TABLE_Y = 0				# [m] nominal y component of the table spawn point

def noisy_pose(nominal_pose, max_offset = 0.2, max_rotation = 2 * 3.14, point = True, rotation = True):
    ##########
    # Given a certain pose, this function adds some "noise" to it.
    # nominal_pose: [Pose] the ideal pose of the object, without any noise. 
    # max_offset:   [float] the maximum distance [m] between the nominal Point and the actual Point along each coordinate.
    #               EXAMPLE: setting max_offset to 5 would not result in a maximum overall distance of 5m, but in a maximum
    #               distance of 5m along each direction, hence the overall distance could be up to sqrt(5^2 + 5^2) which is
    #               slightly more than 7m.
    #               NOTE: set it to 0 to leave nominal_pose.position unaltered.
    # max_rotation: [float] the maximum amplitude [rad] of the rotation around the z axis.
    #               NOTE: set it to 0 to leave nominal_pose.orientation unaltered.
    ##########
    random.seed()    

    noisy_pose = Pose()
    # set the position, adding a random component to the planar components of the pose
    # the vertical (z) component is not altered because of the particular application
    noisy_pose.position.x = nominal_pose.position.x + random.random() * max_offset * random.choice((-1, 1))
    noisy_pose.position.y = nominal_pose.position.y + random.random() * max_offset * random.choice((-1, 1))
    noisy_pose.position.z = nominal_pose.position.z
    # set the orientation (work with euler angles for the sake of simplicity)
    quat = [nominal_pose.orientation.x, nominal_pose.orientation.y, nominal_pose.orientation.z, nominal_pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    # the rotation is performed only around the z axis (yaw), because of the particular geometry of the objects
    # used in this application
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2] + random.random() * max_rotation
    # reconvert the euler angles into a quaternion (required by Pose)
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw) 
    noisy_pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

    return noisy_pose

def spawn_in_pose(obj_name, model_sdf_file, item_pose, namespace = "", with_respect_to = "world"):
    ##########
    # Spawn an object in a given pose.
    # obj_name:         [string] the name that will identify the object in gazebo. Keep in mind that two objects cannot
    #                   have the same name.
    # model_sdf_file:   [string] the complete path to the .sdf file describing the object.
    # item_pose:        [Pose] the desired pose of the object.
    # namespace:        [string] from the documentation: "spawn robot and all ROS interfaces under this namespace".
    # with_respect_to:  [string] from the documentation: "initial_pose is defined relative to the frame of this model/body".
    ##########

    # wait for Gazebo to start
    print("Waiting for gazebo/spawn_sdf_model to start.")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    # read the content of the .sdf model
    with open(model_sdf_file, 'r') as file:
        model = file.read()

    spawn_model(obj_name, model, namespace, item_pose, with_respect_to)
    print(obj_name.capitalize() + " spawned in selected pose.")

def spawn_on_table(obj_name, model_sdf_file):
    ##########
    # Spawn the specified object on top of a "table" model. If no "table" model exists, a new one is spawned.
    # obj_name:         [string] the name that will identify the object in gazebo. Keep in mind that two objects cannot have the 
    #                   same name.
    # model_sdf_file:   [string] the complete path to the .sdf file describing the object.
    ##########

    # spawn the object with name obj_name and described in the model_sdf_file on a table
    table_exists = False
    try:
        # try to get the state (i.e. the pose) of an existing table
        get_model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        table_state = get_model_state("table", "world")
        if table_state.success: 
            # using a flag to set a flag may seem redundant, but it is used since table_state.success does not work as expected
            table_exists = True
    except:
        # an error is displayed in Gazebo every time the table is missing, though everything works as expected (and the table is 
        # correctly spawned)
        pass

    if not table_exists:
        # if a table does not exists, spawn one in a random pose
        print("There is no existing table. A new one is spawned.")
        # the quaternion is hardcoded to make the longer side of the table orthogonal to the world x axis
        pose = noisy_pose(Pose(Point(TABLE_X, TABLE_Y, 0), Quaternion(0, 0, 0.707, 0.707)), max_offset = TABLE_OFFSET, max_rotation = TABLE_ROTATION)
        spawn_in_pose("table", TABLE_SDF_FILE, item_pose = pose)

    # the object is placed at the origin of the table reference frame plus a random offset in order not to place it always
    # at the center of the table
    table_state = get_model_state("table", "world")
    table_state.pose.position.z = TABLE_HEIGHT
    # spawn_in_pose(obj_name, model_sdf_file, item_pose = noisy_pose(table_state.pose), max_offset = OBJECTS_OFFSET))
    spawn_in_pose(obj_name, model_sdf_file, item_pose = noisy_pose(Pose(Point(table_state.pose.position.x - 0.2, table_state.pose.position.y, table_state.pose.position.z),
                                                                        table_state.pose.orientation), max_offset = 0))

def delete(obj_name):
    ##########
    # Delete the specified object.
    # obj_name:     [string] the name of the model to delete.
    ##########
    rospy.wait_for_service("gazebo/delete_model")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    delete_model(obj_name)

if __name__ == '__main__':
    # spawn a table (with the first call to spawn_on_table) with a bottle and a cup on top of it
    spawn_on_table("bottle", BOTTLE_SDF_FILE)
    #spawn_on_table("cup", CUP_SDF_FILE)
    
    '''
    # wait for some time (it would be better to use GetWorldState and the simulated time)
    time.sleep(10) #[s]
    # delete all the models
    delete("cup")
    delete("can")
    try:
        delete("table")
    except rospy.ServiceException:
        pass
	'''