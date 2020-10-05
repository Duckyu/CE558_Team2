#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget, State
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

#rospy.loginfo()
# max_velocity = 2.0
# mode : 'null', 'move', 'takeoff'
position_mask = 0b101111111000
velocity_mask = 0b011111000111
max_velocity = 2.0
max_yaw_rate = 1.570796
angle_snap_resolution = 90 # in degree
height_snap_resolution = 0.25 # in meter
mode = 'null'
setpoint_msg = PositionTarget()
pose = PoseStamped()

# mavros_msgs/CommandBool.srv
# bool value
# ---
# bool success
# uint8 result

def move(mode,x,y,z,q_x,q_y,q_z,q_w):
    control_pub.publish(vel_msg)


def takeoff():
    move()
    arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    arm_result = arming_srv(True)

def pub_sub_setup():
    control_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    joy_sub = rospy.Subscriber("/joy", Joy, joy_callback)
    state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
    local_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_callback)

def joy_callback(self, joy_data):
    #joy_data.axes[]
    #joy_data.buttons[]
    if joy_data.buttons[10] == 1:
        if pose.pose.position.z > 0.05:
            rospy.loginfo("already flying")
        else :
            takeoff()
    elif joy_data.axes[6] != 0 or joy_data.axes[7] != 0:
        setpoint_msg.type_mask = position_mask
        if joy_data.axes[6] != 0:
            current_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            current_angle = R.from_quat(current_quat)
            current_angle = current_rmat.as_euler('zyx', degrees=True)
            yaw_target = current_angle[0]+(joy_data.axes[6] == 1)*angle_snap_resolution-current_angle[0]%angle_snap_resolution
            setpoint_msg.yaw = yaw_target
            setpoint_msg.position = pose.pose.position
        elif joy_data.axes[7] != 0:
            current_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            current_angle = R.from_quat(current_quat)
            current_angle = current_rmat.as_euler('zyx', degrees=True)
            setpoint_msg.yaw = current_angle[0]
            setpoint_msg.position = pose.pose.position
            setpoint_msg.position.z =  pose.pose.position.z + height_snap_resolution * joy_data.axes[7]
    elif joy_data.axes[0] != 0 or joy_data.axes[1] != 0 or joy_data.axes[3] != 0 or joy_data.axes[4] != 0:
        setpoint_msg.type_mask = velocity_mask
        setpoint_msg.velocity.x = max_velocity
        setpoint_msg.velocity.y = max_velocity
        setpoint_msg.velocity.z = max_velocity
        setpoint_msg.yaw_rate = - max_velocity * joy_data.axes[0]
    else:
        setpoint_msg.type_mask = velocity_mask
        setpoint_msg.velocity.x = 0
        setpoint_msg.velocity.y = 0
        setpoint_msg.velocity.z = 0
        setpoint_msg.yaw_rate = 0

def state_callback(self, state_data):


def local_position_callback(self, local_position_data):
    global pose = local_position_data

if __name__ == '__main__':
    rospy.init_node('dualshock_UAV', anonymous=True)
    rate = rospy.Rate(10)
    pub_sub_setup()

    try:
        control_pub.publish
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Node terminated by pressing Ctrl+C!')
        sys.exit(0)