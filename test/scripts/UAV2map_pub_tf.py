#!/usr/bin/env python

import rospy
import sys

import tf
import tf_conversions
import tf2_ros

from std_msgs.msg import Header
# from geometry_msgs.msg import PoseStamped, TransformStamped
from geometry_msgs.msg import Pose, TransformStamped
from gazebo_msgs.msg import ModelStates

# pose = PoseStamped()
pose = Pose()

def sub_setup():
    # local_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_callback)
	model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

def tf_pub():
    global pose
    br = tf2_ros.TransformBroadcaster()
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    tf_temp = TransformStamped()
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link"
    static_transformStamped.child_frame_id = "velodyne"

    static_transformStamped.transform.translation.x = 0.08 
    static_transformStamped.transform.translation.y = 0 
    static_transformStamped.transform.translation.z = 0.1 

    quat = tf.transformations.quaternion_from_euler(0, 0.5235988, 0)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    # tf_temp.transform.translation.x = pose.pose.position.x
    # tf_temp.transform.translation.y = pose.pose.position.y
    # tf_temp.transform.translation.z = pose.pose.position.z
    # tf_temp.transform.rotation = pose.pose.orientation
    tf_temp.transform.translation.x = pose.position.x
    tf_temp.transform.translation.y = pose.position.y
    tf_temp.transform.translation.z = pose.position.z
    tf_temp.transform.rotation = pose.orientation
    tf_temp.header.stamp = rospy.Time.now()
    tf_temp.header.frame_id = 'map'
    tf_temp.child_frame_id = 'base_link'
    br.sendTransform(tf_temp)
    broadcaster.sendTransform(static_transformStamped)
    return

# def local_position_callback(data):
#     global pose
#     pose = data
#     return

def model_states_callback(data):
    global pose
    index = 0
    for model in data.name:
    	if model == "iris_VLP16":
    		pose = data.pose[index]
    	else :
    		index+=1
    return

if __name__ == '__main__':
    # global pose
    # global setpoint_msg
    rospy.init_node('cvtPose2TF', anonymous=True)
    rate = rospy.Rate(50)
    sub_setup()

    while not rospy.is_shutdown():
        try:
            # rospy.loginfo("try")
            tf_pub()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('Node terminated by pressing Ctrl+C!')
            sys.exit(0)