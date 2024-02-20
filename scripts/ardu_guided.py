#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, RCOut
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

# グローバル変数の定義
current_state = State()
current_rcout = RCOut()

def state_cb(msg):
    global current_state
    current_state = msg

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
    rospy.loginfo("R: %5.2f, P: %5.2f, Y: %5.2f" % (roll, pitch, yaw))

def main():
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

    # FCU接続を待つ
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.spin()
        rate.sleep()

    # グローバルポジションの原点を設定
    geo = GeoPointStamped()
    geo.position.latitude = 33.595270
    geo.position.longitude = 130.215496
    set_gp_origin_pub.publish(geo)

    pose = PoseStamped()
    pose.pose.position.x = 1
    pose.pose.position.y = 0
    pose.pose.position.z = 1.5

    # セットポイントを送信
    for i in range(100):
        local_pos_pub.publish(pose)
        rospy.spin()
        rate.sleep()

    # GUIDEDモードに設定
    try:
        set_mode_client(custom_mode="GUIDED")
        rospy.loginfo("Guided enabled")
    except rospy.ServiceException as e:
        rospy.logerr("Set mode failed: %s" % e)

    # 機体のアーム
    try:
        arming_client(True)
        rospy.loginfo("Vehicle armed")
    except rospy.ServiceException as e:
        rospy.logerr("Arming failed: %s" % e)

    # 離陸
    try:
        takeoff_client(altitude=1.5)
        rospy.loginfo("Takeoff sent")
    except rospy.ServiceException as e:
        rospy.logerr("Takeoff failed: %s" % e)

    rospy.sleep(5)

    # メインループ
    start_time = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < rospy.Duration(120.0):
        time_sec = (rospy.Time.now() - start_time).to_sec()
        pose.pose.position.x = 1.0 * math.cos(time_sec)
        pose.pose.position.y = 1.0 * math.sin(time_sec)
        pose.pose.position.z = 1.5

        local_pos_pub.publish(pose)

        rospy.spin()
        rospy.sleep(0.1)

    # 着陸
    try:
        land_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        land_client()
        rospy.loginfo("Land sent")
    except rospy.ServiceException as e:
        rospy.logerr("Landing failed: %s" % e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
