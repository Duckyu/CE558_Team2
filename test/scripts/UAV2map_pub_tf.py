#!/usr/bin/env python

import rospy
import sys

import tf_conversions
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped

pose = PoseStamped()

def sub_setup():
    local_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_callback)

def tf_pub():
    global pose
    br = tf2_ros.TransformBroadcaster()
    tf_temp = TransformStamped()
    tf_temp.transform.translation.x = pose.pose.position.x
    tf_temp.transform.translation.y = pose.pose.position.y
    tf_temp.transform.translation.z = pose.pose.position.z
    tf_temp.transform.rotation = pose.pose.orientation
    tf_temp.header = pose.header
    # tf_temp.header.frame_id = 'map'
    tf_temp.child_frame_id = 'base_link'

    br.sendTransform(tf_temp)
    # control_pub = rospy.Publisher("/tf", TFMessage, queue_size=100)
    # control_pub.publish(cvt_tf)

def local_position_callback(data):
    global pose
    pose = data

if __name__ == '__main__':
    # global pose
    # global setpoint_msg
    rospy.init_node('cvtPose2TF', anonymous=True)
    rate = rospy.Rate(100)
    sub_setup()

    while not rospy.is_shutdown():
        try:
            # rospy.loginfo("try")
            tf_pub()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('Node terminated by pressing Ctrl+C!')
            sys.exit(0)