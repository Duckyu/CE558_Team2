#!/usr/bin/env python

import rospy
import sys

import tf
import tf_conversions
import tf2_ros

from std_msgs.msg import Header
# from geometry_msgs.msg import PoseStamped, TransformStamped
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
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
    static_transformStamped_up = TransformStamped()
    static_transformStamped_front = TransformStamped()
    #tf.transformations.quaternion_from_euler(0, -1.570796, 0)

    static_transformStamped_up.header.stamp = rospy.Time.now()
    static_transformStamped_up.header.frame_id = "base_link"
    static_transformStamped_up.child_frame_id = "up_camera_link"

    static_transformStamped_up.transform.translation.x = 0 
    static_transformStamped_up.transform.translation.y = 0 
    static_transformStamped_up.transform.translation.z = 0.05 

    quat = tf.transformations.quaternion_from_euler(-3.141592, -3.141592, 1.570796)
    static_transformStamped_up.transform.rotation.x = quat[0]
    static_transformStamped_up.transform.rotation.y = quat[1]
    static_transformStamped_up.transform.rotation.z = quat[2]
    static_transformStamped_up.transform.rotation.w = quat[3]

    static_transformStamped_front.header.stamp = rospy.Time.now()
    static_transformStamped_front.header.frame_id = "base_link"
    static_transformStamped_front.child_frame_id = "front_camera_link"

    static_transformStamped_front.transform.translation.x = 0.1 
    static_transformStamped_front.transform.translation.y = 0 
    static_transformStamped_front.transform.translation.z = 0

    quat = tf.transformations.quaternion_from_euler(1.570796, -3.141592, 1.570796)
    static_transformStamped_front.transform.rotation.x = quat[0]
    static_transformStamped_front.transform.rotation.y = quat[1]
    static_transformStamped_front.transform.rotation.z = quat[2]
    static_transformStamped_front.transform.rotation.w = quat[3]

    tf_temp.transform.translation.x = pose.position.x
    tf_temp.transform.translation.y = pose.position.y
    tf_temp.transform.translation.z = pose.position.z
    if (pose.orientation.x * pose.orientation.y * pose.orientation.z * pose.orientation.w == 0):
        tf_temp.transform.rotation.x = 0
        tf_temp.transform.rotation.y = 0
        tf_temp.transform.rotation.z = 0
        tf_temp.transform.rotation.w = 1
    else:
        tf_temp.transform.rotation = pose.orientation
    tf_temp.transform.rotation = pose.orientation
    tf_temp.header.stamp = rospy.Time.now()
    tf_temp.header.frame_id = 'map'
    tf_temp.child_frame_id = 'base_link'
    br.sendTransform(tf_temp)
    broadcaster.sendTransform(static_transformStamped_up)
    broadcaster.sendTransform(static_transformStamped_front)
    return

def model_pub():
    global pose
    msg = PoseStamped()
    msg.pose.position.x = pose.position.x
    msg.pose.position.y = pose.position.y
    msg.pose.position.z = pose.position.z

    msg.pose.orientation.x = pose.orientation.x
    msg.pose.orientation.y = pose.orientation.y
    msg.pose.orientation.z = pose.orientation.z
    msg.pose.orientation.w = pose.orientation.w

    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    pub = rospy.Publisher("/UAV", PoseStamped, queue_size=10)
    pub.publish(msg)


def model_states_callback(data):
    global pose
    index = -1
    model_index = -1 
    for model in data.name:
        index+=1    	
        if model == "iris_front_up_depth_camera":
            model_index = index
            pose = data.pose[model_index]
    if model_index == -1:
        print "check model name"
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
            model_pub()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('Node terminated by pressing Ctrl+C!')
            sys.exit(0)
