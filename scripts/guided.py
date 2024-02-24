
import rospy
import math
import px4_sim_pkg.MavrosNode as MavrosNode

rospy.init_node('offb_node', anonymous=True)

rate = rospy.Rate(20.0)
mav = MavrosNode.MavrosNode()
rospy.loginfo("Initializing ...")
# FCU接続を待つ
while not rospy.is_shutdown() and not mav.current_state.connected:
    rospy.loginfo("current_state.connected: %s" % (mav.current_state.connected))
    rospy.sleep(0.1)
    
rospy.loginfo("Waiting for connection...")

# グローバルポジションの原点を設定
rospy.loginfo("Set Global Position ...")
latitude = 33.595270
longitude = 130.215496
mav.set_gp_position(latitude, longitude)


# セットポイントを送信
rospy.loginfo("Sending setpoint ...")
for i in range(100):
    mav.set_local_position(1, 0, 1.5)
    rate.sleep()
rospy.loginfo("Done sending setpoint ...")

# GUIDEDモードに設定
mav.set_drone_to_guided_mode()

# 機体のアーム
mav.arm_vehicle()

# 離陸
mav.vehicle_takeoff(1.5)

rospy.sleep(5)


# メインループ
start_time = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < rospy.Duration(60.0):
    time_sec = (rospy.Time.now() - start_time).to_sec()
    x = 1.0 * math.cos(time_sec)
    y = 1.0 * math.sin(time_sec)
    z = 1.5
    mav.set_local_position(x, y, z)
    rospy.sleep(0.1)

# 着陸
mav.send_land_command()

