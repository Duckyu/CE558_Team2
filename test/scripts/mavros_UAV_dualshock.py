#!/usr/bin/env python

import rospy
import sys
from mavros_msgs.srv import CommandBool, SetMode
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget, State, ExtendedState
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
takeoff_height = 1.5
# mode = 'null'
setpoint_msg = PositionTarget()
hover_flag = False
hover = PositionTarget()
pose = PoseStamped()
state = State()
ext_state = ExtendedState()

# mavros_msgs/CommandBool.srv
# bool value
# ---
# bool success
# uint8 result
def sub_setup():
    joy_sub = rospy.Subscriber("/joy", Joy, joy_callback)
    state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
    ext_state_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, ext_state_callback)
    local_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_callback)

def move(control_msg):
    control_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    control_pub.publish(control_msg)

def takeoff():
    global pose
    # setpoint_msg = PositionTarget()
    setpoint_msg.type_mask = position_mask
    setpoint_msg.position = pose.pose.position
    setpoint_msg.position.z = setpoint_msg.position.z + takeoff_height
    move(setpoint_msg)
    arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)#mavros_msgs.srv.
    arm_result = arming_srv(True)
    mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)#mavros_msgs.srv.
    mode_result = mode_srv(0,"OFFBOARD")

def joy_callback(joy_data): # self, 
    global setpoint_msg
    global pose
    global hover_flag
    #joy_data.axes[]
    #joy_data.buttons[]
    if joy_data.buttons[10] == 1:
        if ext_state.landed_state == ExtendedState().LANDED_STATE_ON_GROUND:
            rospy.loginfo("attempting takeoff")
            takeoff()
        else:
            rospy.loginfo("already flying : %lf", pose.pose.position.z)
    elif joy_data.axes[6] != 0 or joy_data.axes[7] != 0:
        setpoint_msg.type_mask = position_mask
        rospy.loginfo("type_mask : %d", setpoint_msg.type_mask)
        if joy_data.axes[6] != 0:
            current_quat = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
            current_rmat = R.from_quat(current_quat)
            current_angle = current_rmat.as_euler('zyx', degrees=True)
            yaw_target = current_angle[0]+(joy_data.axes[6] == 1)*angle_snap_resolution-current_angle[0]%angle_snap_resolution
            setpoint_msg.yaw = yaw_target
            setpoint_msg.position = pose.pose.position
        elif joy_data.axes[7] != 0:
            current_quat = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
            current_rmat = R.from_quat(current_quat)
            current_angle = current_rmat.as_euler('zyx', degrees=True)
            setpoint_msg.yaw = current_angle[0]
            setpoint_msg.position = pose.pose.position
            setpoint_msg.position.z =  pose.pose.position.z + height_snap_resolution * joy_data.axes[7]
    elif joy_data.axes[0] != 0 or joy_data.axes[1] != 0 or joy_data.axes[3] != 0 or joy_data.axes[4] != 0:
        setpoint_msg.type_mask = velocity_mask
        rospy.loginfo("type_mask : %d", setpoint_msg.type_mask)
        setpoint_msg.velocity.x = max_velocity * joy_data.axes[4]
        setpoint_msg.velocity.y = max_velocity * joy_data.axes[3]
        setpoint_msg.velocity.z = max_velocity * joy_data.axes[1]
        setpoint_msg.yaw_rate = max_velocity * joy_data.axes[0]
    else:
        hover_flag = True

        
def state_callback(data):# self, 
    global state
    state = data

def ext_state_callback(data):# self, 
    global ext_state
    ext_state = data

def local_position_callback(data): # self, 
    global pose
    global hover_flag
    pose = data
    if hover_flag == True:
        hover_flag = False
        hover.type_mask = position_mask
        hover.position = pose.pose.position
    # rospy.loginfo("poss x,y,z: %lf, %lf, %lf", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)

if __name__ == '__main__':
    # global pose
    # global setpoint_msg
    rospy.init_node('dualshock_UAV', anonymous=True)
    setpoint_msg.coordinate_frame = 8;
    rate = rospy.Rate(30)
    sub_setup()

    while not rospy.is_shutdown():
        try:
            move(setpoint_msg)
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('Node terminated by pressing Ctrl+C!')
            sys.exit(0)