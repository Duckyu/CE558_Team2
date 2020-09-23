#!/usr/bin/env python

import rospy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

#rospy.loginfo()
# max_velocity = 2.0
# mode : 'null', 'move', 'takeoff'
position_mask = 0b101111111000
velocity_mask = 0b011111000111
max_velocity = 2.0
angle_snap_resolution = 90 # in degree
height_snap_resolution = 0.25 # in meter
mode = 'null'
setpoint_msg = PositionTarget()
pose = PoseStamped()

def move(mode,x,y,z,q_x,q_y,q_z,q_w):

def takeoff():
    move()

def pub_sub_setup():
    control_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    joy_sub = rospy.Subscriber("/joy", Joy, joy_callback)
    local_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_callback)

def joy_callback(joy_data):
    #joy_data.axes[]
    #joy_data.buttons[]
    if joy_data.buttons[10] == 1:
        if pose.pose.position.z > 0.05:
            rospy.loginfo("already flying")
        else :
            takeoff()
    elif joy_data.axes[6] != 0 or joy_data.axes[7] != 0:
        setpoint_msg.type_mask = position_mask
        if joy_data.axes[6] != 0
            current_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            current_angle = R.from_quat(current_quat)
            current_angle = current_rmat.as_euler('zyx', degrees=True)
            yaw_target = current_angle[0]+(joy_data.axes[6] == 1)*angle_snap_resolution-current_angle[0]%angle_snap_resolution
            setpoint_msg.yaw = yaw_target
        elif joy_data.axes[7] != 0




def local_position_callback(local_position_data):
    global pose = local_position_data


def listener():
    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('dualshock_UAV', anonymous=True)
    rate = rospy.Rate(10)
    pub_sub_setup()

    try:
        talker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Node terminated by pressing Ctrl+C!')
        sys.exit(0)

