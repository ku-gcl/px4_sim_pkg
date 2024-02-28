# function.py
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import math
from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_from_euler


# GYM_OFFSET = 0.0

current_state = State()
# current_pose = PoseStamped()
# current_heading = Float64()

def state_cb(msg):
    global current_state
    current_state = msg
    rospy.loginfo(f"Connected: {msg.connected}, Armed: {msg.armed}")
    return current_state

def rcout_cb(msg):
    global current_rcout
    current_rcout = msg
    # print("RC Channels 1-4: [%d, %d, %d, %d]" % (current_rcout.channels[0], current_rcout.channels[1], current_rcout.channels[2], current_rcout.channels[3]))

def imu_cb(msg):
    global imu_data
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    # rospy.loginfo("Roll: %f, Pitch: %f, Yaw: %f" % (roll, pitch, yaw))
    # rospy.loginfo("R: %5.2f, P: %5.2f, Y: %5.2f" % (roll, pitch, yaw))


def pose_cb(msg):
    global current_pose
    current_pose = msg
    rospy.loginfo(f"x: {msg.pose.position.x}, y: {msg.pose.position.y}, z: {msg.pose.position.z}")

def heading_cb(msg):
    global current_heading
    current_heading = msg
    rospy.loginfo(f"Current heading: {msg.data}")


# destination
def set_heading(heading, pose, GYM_OFFSET):
    heading = -heading + 90 - GYM_OFFSET
    yaw = math.radians(heading)
    pose.pose.orientation = quaternion_from_euler(0, 0, yaw)
    return pose

def set_destination(x, y, z, GYM_OFFSET):
    deg2rad = math.pi / 180
    X = x * math.cos(-GYM_OFFSET * deg2rad) - y * math.sin(-GYM_OFFSET * deg2rad)
    Y = x * math.sin(-GYM_OFFSET * deg2rad) + y * math.cos(-GYM_OFFSET * deg2rad)
    Z = z
    pose = PoseStamped()
    pose.pose.position.x = X
    pose.pose.position.y = Y
    pose.pose.position.z = Z
    rospy.loginfo(f"Destination set to x: {X}, y: {Y}, z: {Z}")
    return pose

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    qw = cy * cr * cp + sy * sr * sp
    qx = cy * sr * cp - sy * cr * sp
    qy = cy * cr * sp + sy * sr * cp
    qz = sy * cr * cp - cy * sr * sp

    return {'w': qw, 'x': qx, 'y': qy, 'z': qz}
