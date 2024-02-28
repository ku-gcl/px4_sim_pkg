import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, RCOut
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from scripts.archive.command import *
from scripts.archive.function import state_cb, rcout_cb, imu_cb, pose_cb, heading_cb
from scripts.archive.function import set_heading, set_destination, quaternion_from_euler



GYM_OFFSET = 0.0

current_state = State()
current_pose = PoseStamped()
current_heading = Float64()
current_rcout = RCOut()



def main():
    global current_state, current_rcout, current_pose, current_heading

    rospy.init_node('offb_node', anonymous=True)
    
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    set_gp_origin_pub = rospy.Publisher("mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
    takeoff_client = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)
    rcout_sub = rospy.Subscriber("mavros/rc/out", RCOut, rcout_cb)
    imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, imu_cb)

    rate = rospy.Rate(20.0)  # MUST be > 2Hz

    rospy.loginfo("Initializing ...")
    # FCU接続を待つ
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("current_state.connected: %s" % (current_state.connected))
        rospy.sleep(0.1)

    rospy.loginfo("Waiting for connection...")

    # グローバルポジションの原点を設定
    rospy.loginfo("Set Global Position ...")
    geo = GeoPointStamped()
    geo.position.latitude = 33.595270
    geo.position.longitude = 130.215496
    set_gp_origin_pub.publish(geo)
    rospy.loginfo("Done Global Position ...")

    pose = PoseStamped()
    pose.pose.position.x = 1
    pose.pose.position.y = 0
    pose.pose.position.z = 1.5

    # セットポイントを送信
    rospy.loginfo("Sending setpoint ...")
    for i in range(100):
        local_pos_pub.publish(pose)
        # rospy.spin()
        rate.sleep()
    rospy.loginfo("Done sending setpoint ...")

    # GUIDEDモードに設定
    set_drone_to_guided_mode()

    # 機体のアーム
    arm_vehicle()

    # 離陸
    vehicle_takeoff(1.5)

    rospy.sleep(5)

    # メインループ
    start_time = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < rospy.Duration(120.0):
        time_sec = (rospy.Time.now() - start_time).to_sec()
        pose.pose.position.x = 1.0 * math.cos(time_sec)
        pose.pose.position.y = 1.0 * math.sin(time_sec)
        pose.pose.position.z = 1.5

        local_pos_pub.publish(pose)

        # rospy.spin()
        rospy.sleep(0.1)

    # 着陸
    send_land_command()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
